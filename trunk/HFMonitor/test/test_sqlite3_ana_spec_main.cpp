// -*- mode: C++; c-basic-offset: 2; indent-tabs-mode: nil  -*-
// $Id$
//
// Copyright 2010-2014 Christoph Mayer
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#include <cmath>
#include <deque>
#include <iostream>
#include <fstream>
#include <iterator>
#include <map>
#include <numeric>
#include <boost/format.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Text_Display.H>
#include <FL/Fl_Native_File_Chooser.H>
#include <FL/Fl_Select_Browser.H>

/* Ensure we are using opengl's core profile only */
#define GL3_PROTOTYPES 1
#include <FL/Fl_Gl_Window.H>
#include <FL/gl.h>
// #include <FL/glu.h>
#if defined(__APPLE__)
#  include <OpenGL/gl3.h> // defines OpenGL 3.0+ functions
#endif

#include "Spectrum.hpp"
#include "logging.hpp"
#include "run.hpp"
#include "carrier_monitoring/background_estimator.hpp"
#include "FFTProcessor/Filter.hpp"

#include "db/sqlite3.hpp"

class ShaderSource {
public:
  ShaderSource(std::string fileName)
    : _count(0) {
    std::ifstream ifs(fileName);
    std::string line;
    _str.clear();
    while (std::getline(ifs, line)) {
      std::copy(line.begin(), line.end(), std::back_inserter(_str));
      _str.push_back('\n');
      std::cout << "line = " << line << std::endl;        
    }
    _str.push_back('\0');
    _count = 1;
    _pstr = &_str[0];
  }
  
  ~ShaderSource() {}
  
  GLsizei  count() const { return _count; }
  const GLchar** str() const { 
    return const_cast<const GLchar**>(&_pstr);
  }
  const GLint* length() const { return NULL; }
  
private:
  GLsizei             _count;
  std::vector<GLchar> _str;
   GLchar* _pstr;
} ;

class Shader {
public:
  Shader(GLenum shaderType,
         std::string sourceFileName)
    : id_(glCreateShader(shaderType)) {
    
    if (id_ == 0)
      std::cout << "Error" << std::endl;

    const ShaderSource ss(sourceFileName);
    glShaderSource(id_, ss.count(), ss.str(), ss.length());
    
    glCompileShader(id_);

    GLint compile_ok(0);
    glGetShaderiv(id_, GL_COMPILE_STATUS, &compile_ok);
    std::cout << "compile_ok= " << compile_ok << std::endl;

    if (compile_ok != GL_TRUE) {
      GLchar infoLog[1024] = { 0 };
      GLsizei maxLength(1024);
      GLsizei len(0);
      glGetShaderInfoLog(id_, maxLength, &len, infoLog);
      std::cout << "Log: " << infoLog << std::endl;
    }
  }

  ~Shader() {
    glDeleteShader(id_);
  }

  GLint id() const { return id_; }

protected:
private:
  GLint id_;
} ;

class Program {
public:
  typedef boost::shared_ptr<Program> sptr;

  static sptr make() {
    return sptr(new Program());
  }

  ~Program() {      
    glDeleteProgram(id_);
  }

  Program& use() {
    glUseProgram(id_);
    return *this;
  }

  GLint getAttribLocation(std::string attribName) {
    return glGetAttribLocation(id_, attribName.c_str());
  }
  void bindAttribLocation(GLuint index, std::string attribName) {
    glBindAttribLocation(id_, index, attribName.c_str());
  }
  void bindFragDataLocation(GLuint colorNumber, std::string name) {
    glBindFragDataLocation(id_, colorNumber, name.c_str());
  }
  GLint getUniformLocation(std::string attribName) {
    return glGetUniformLocation(id_, attribName.c_str());
  }

  Program& attachShader(GLint shaderId) {
    glAttachShader(id_, shaderId);
    return *this;
  }
  Program& link() {
    glLinkProgram(id_);
    GLint compile_ok(0);
    glGetProgramiv(id_, GL_LINK_STATUS, &compile_ok);
    std::cout << "compile_ok= " << compile_ok << std::endl;
    
    glValidateProgram(id_);
    glGetProgramiv(id_, GL_VALIDATE_STATUS, &compile_ok);
    std::cout << "validate_ok= " << compile_ok << std::endl;
    
    const GLsizei maxCount(64);
    GLuint  shaders[maxCount];
    GLsizei count(0);
    glGetAttachedShaders(id_, maxCount, &count, shaders);
    for (int i=0; i<count; ++i)
      glDetachShader(id_, shaders[i]);
    
    return *this;
  }
    
  GLint id() const { return id_; }
protected:
private:
  Program()
    : id_(glCreateProgram()) {}
  Program(const Program&);
  Program& operator=(const Program&);
  
  GLint id_;
} ;

class Texture {
public:
  typedef boost::shared_ptr<Texture> sptr;

  static sptr make() {
    return sptr(new Texture());
  }

  ~Texture() {
    glDeleteTextures(1, &id_);
    id_ = 0;
  }
  
  void bind() {
    glBindTexture(GL_TEXTURE_2D, id_);
  }

  GLuint id() const { return id_; }

protected:
private:
  Texture()
    : id_(0) {
    glGenTextures(1, &id_);
  }
  Texture(const Texture&);
  Texture& operator=(const Texture&);
  GLuint id_;
} ;

  // adapted from palette.c \in linrad
unsigned char colorScale(size_t index) {
  static unsigned char color_scale[]={ 
    0, 36,37,38,39,40,41,42,43,44,
    45,46,47,48,49,50,51,52,53,54,
    55,15,15};
  static size_t len(sizeof(color_scale)/sizeof(unsigned char));
  return color_scale[(index>len) ? len-1 : index];
}
// adapted from palette.c \in linrad
unsigned char svgaPalette(size_t index) {
  static unsigned char svga_palette[] = {
    //    0       |      1       |     2        |     3
    0x00,0x00,0x00,0x2a,0x00,0x00,0x00,0x2a,0x00,0x2a,0x2a,0x00,
    //    4       |      5       |     6        |     7
    0x00,0x00,0x2a,0x2a,0x00,0x2a,0x00,0x15,0x2a,0x2a,0x2a,0x2a,
    //    8       |      9       |     10       |     11
    0x15,0x15,0x15,0x3f,0x15,0x15,0x15,0x3f,0x15,0x3f,0x3f,0x15,
    //    12      |      13      |     14       |     15
    0x15,0x15,0x3f,0x3f,0x15,0x3f,0x15,0x3f,0x3f,0x3f,0x3f,0x3f,
    //    16      |      17      |     18       |     19
    0x00,0x17,0x00,0x05,0x05,0x05,0x08,0x08,0x08,0x0b,0x0b,0x0b,
    //    20      |      21      |     22       |     23
    0x0e,0x0e,0x0e,0x11,0x11,0x11,0x14,0x14,0x14,0x18,0x18,0x18,
    //    24      |      25      |     26       |     27
    0x1c,0x1c,0x1c,0x20,0x20,0x20,0x24,0x24,0x24,0x28,0x28,0x28,
    //    28      |      29      |     30       |     31
    0x2d,0x2d,0x2d,0x32,0x32,0x32,0x38,0x38,0x38,0x08,0x18,0x1a,
    //    32      |      33      |     34       |     35
    0x20,0x3f,0x20,0x12,0x00,0x00,0x17,0x00,0x00,0x00,0x00,0x17,
    //    36      |      37      |     38       |     39
    0x08,0x00,0x00,0x10,0x00,0x00,0x18,0x00,0x00,0x1a,0x0a,0x00,
    //    40      |      41      |     42       |     43
    0x1c,0x0f,0x00,0x1d,0x16,0x00,0x17,0x1c,0x00,0x14,0x21,0x00,
    //    44      |      45      |     46       |     47
    0x05,0x23,0x00,0x00,0x28,0x0a,0x00,0x28,0x1e,0x00,0x23,0x25,
    //    48      |      49      |     50       |     51
    0x00,0x21,0x30,0x00,0x1c,0x32,0x00,0x16,0x34,0x00,0x0f,0x3a,
    //    52      |      53      |     54       |     55
    0x0a,0x0a,0x3f,0x14,0x14,0x3f,0x1f,0x1f,0x3f,0x2b,0x2b,0x3f,
      //    56      |      57      |     58       |     59
    0x0e,0x29,0x22,0x12,0x12,   0,0x30,0x30,0x3f,         0,0,0};
  static size_t len(sizeof(svga_palette)/sizeof(unsigned char));
  return svga_palette[(index>len) ? len-1 : index];
}

class OpenGlPlot : public Fl_Gl_Window {
public:
  OpenGlPlot(int X, int Y, int W, int H, const char *L=0)
    : Fl_Gl_Window(X, Y, W, H, L)
    , texture_()
    , textureCol_()
    , vao_(0)
    , prog_() {
      mode(FL_RGB | FL_DOUBLE | FL_OPENGL3);
      vbo_[0] = vbo_[1] = 0;
  }
  virtual ~OpenGlPlot() {

  }

  Texture::sptr texture() { return texture_; }  

protected:
  void GlInit() {
    static bool first_time = true;
    if (first_time) {
      first_time = false;      
      
      texture_ = Texture::make();
      glActiveTexture(GL_TEXTURE0);
      texture_->bind();

#if 0            
      {
        const size_t imgWidth  = 256;
        const size_t imgHeight = 256;
        GLubyte img[imgHeight*imgWidth*1] = { 0 };
        for (size_t j=0; j<imgHeight; ++j)
          for (size_t i=0; i<imgWidth; ++i)
            img[j*imgWidth*1 + i*1 + 0] = GLubyte(256*double(j)/double(imgHeight) * double(i)/double(imgWidth));
        
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, imgWidth, imgHeight, 0, GL_RED, GL_UNSIGNED_BYTE, img);
      }
      glGenerateMipmap(GL_TEXTURE_2D);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); 
#endif
      // 

      textureCol_ = Texture::make();
      glActiveTexture(GL_TEXTURE1);
      textureCol_->bind();
      {
        const size_t imgWidth  = 256;
        const size_t imgHeight = 1;
        GLubyte img[imgHeight*imgWidth*3] = { 0 };
        for (size_t j=0; j<imgHeight; ++j) {
          for (size_t i=0; i<imgWidth; ++i) {
            const double x = double(i)/double(imgWidth);
            const size_t colorMapIndex(colorScale((unsigned char)(x*22)));
            img[j*imgWidth*3 + i*3 + 0] = GLubyte(svgaPalette(3*colorMapIndex+2)<<2);
            img[j*imgWidth*3 + i*3 + 1] = GLubyte(svgaPalette(3*colorMapIndex+1)<<2);
            img[j*imgWidth*3 + i*3 + 2] = GLubyte(svgaPalette(3*colorMapIndex+0)<<2);
          }
        }        
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, imgWidth, imgHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, img);
      }

      glGenerateMipmap(GL_TEXTURE_2D);
      
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); 
    }
  }


  void draw() {

    std::cout << "draw valid= " << int(valid()) << std::endl;
    if (!valid()) {
//       valid(1);
      GlInit();

      const GLfloat vertices[6][2] = {
        {-1.0, -1.0},
        {+1.0, -1.0},
        {+1.0, +1.0},

        {+1.0, +1.0},
        {-1.0, +1.0},
        {-1.0, -1.0}
      };
      const GLfloat texVertices[6][2] = {
        {0.0, 0.0},
        {1.0, 0.0},
        {1.0, 1.0},

        {1.0, 1.0},
        {0.0, 1.0},
        {0.0, 0.0}
      };
            
      glGenVertexArrays(1, &vao_);
      glBindVertexArray(vao_);

      glGenBuffers(2, vbo_);
      glBindBuffer(GL_ARRAY_BUFFER, vbo_[0]);
      glBufferData(GL_ARRAY_BUFFER, 12*sizeof(GLfloat), vertices, GL_STATIC_DRAW);
      glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, NULL);
      glEnableVertexAttribArray(0);

      glBindBuffer(GL_ARRAY_BUFFER, vbo_[1]);
      glBufferData(GL_ARRAY_BUFFER, 12*sizeof(GLfloat), texVertices, GL_STATIC_DRAW);
      glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, NULL);
      glEnableVertexAttribArray(1);        

      glBindVertexArray(0);
      
#if 1
      Shader vertexShader  (GL_VERTEX_SHADER,   "../test/vertexShader.txt");
      CheckErr("vtx");
      Shader fragmentShader(GL_FRAGMENT_SHADER, "../test/fragShader.txt");
      CheckErr("frag");

      prog_ = Program::make();
      CheckErr("make");
      prog_->attachShader(vertexShader.id());
      CheckErr("attach vtx");
      prog_->attachShader(fragmentShader.id());
      CheckErr("attach frag");

      prog_->bindAttribLocation(0, "pos2d");
      CheckErr("bind pos2d");
      prog_->bindAttribLocation(1, "tex2d");
      CheckErr("bind tex2d");

      prog_->bindFragDataLocation(0, "fragColor");

      prog_->link();

      CheckErr("link");

      prog_->use();
      CheckErr("use");

      
      GLint loc1 = prog_->getUniformLocation("sMin");
      std::cout << "sMin= " << loc1 << std::endl;
      glUniform1f(loc1, 0.0);
      CheckErr("XX");
      loc1 = prog_->getUniformLocation("sMax");
      std::cout << "sMax= " << loc1 << std::endl;
      glUniform1f(loc1, 1.0);
      CheckErr("XX");

      std::cout << "VERSION= " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;
      std::cout << "FragDataLoc= " << glGetFragDataLocation(prog_->id(), "fragColor") << std::endl;

      glActiveTexture(GL_TEXTURE0);
      CheckErr("glActiveTexture");
//       texture_->bind();

      loc1 = prog_->getUniformLocation("spec");
      std::cout << "spec= " << loc1 << " " << texture_->id() << std::endl;
      glUniform1i(loc1, 0);
      CheckErr("glUniform1i spec");

      glActiveTexture(GL_TEXTURE1);
//       textureCol_->bind();

      loc1 = prog_->getUniformLocation("colLUT");
      std::cout << "colLUT= " << loc1 << " " << textureCol_->id() << std::endl;
      glUniform1i(loc1, 1);
      CheckErr("glUniform1i colLUT");

      GLuint MatrixID = glGetUniformLocation(prog_->id(), "MVP");
      GLfloat mvp[4][4];
      for (int i=0; i<4; ++i)
        for (int j=0; j<4; ++j)
          mvp[i][j] = 0;
      {
//         const float l = -2; const float r = +5;
//         const float b = -2; const float t = +5;
//         const float f = -1; const float n = +1;
//         mvp[0][0] = +2/(r - l); mvp[0][3] = -(r+l)/(r-l);
//         mvp[1][1] = +2/(t - b); mvp[1][3] = -(t+b)/(t-b);
//         mvp[2][2] = -2/(f - n); mvp[2][3] = -(f+n)/(f-n);
        mvp[0][0] = 0.9;
        mvp[1][1] = 0.9;
        mvp[2][2] = 1;
        mvp[3][3] = 1;
      }
      glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &mvp[0][0]);
      CheckErr("glUniformMatrix4fv");

#endif
    }

    glClearColor(0.5, 0.5, 0.5, 0.0);
    CheckErr("clearcolor");
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    CheckErr("clear");

    prog_->use();
    CheckErr("prog use");

    glActiveTexture(GL_TEXTURE0);
//     texture_->bind();
    CheckErr("bind tex0");

    glActiveTexture(GL_TEXTURE1);
//     textureCol_->bind();
    CheckErr("bind tex1");

    glBindVertexArray(vao_);
    CheckErr("bind vao");
    glDrawArrays(GL_TRIANGLES, 0, 6);
    CheckErr("drawarrays");
    glBindVertexArray(0);
    CheckErr("bind vao 0");

    CheckErr("finish");
  }

  void CheckErr(std::string msg) const {
    const GLint err = glGetError();
    switch (err) {
    case (GL_NO_ERROR):
      msg += " GL_NO_ERROR ";
      break;
    case (GL_INVALID_ENUM):
      msg += " GL_INVALID_ENUM";
      break;
    case (GL_INVALID_VALUE):
      msg += " GL_INVALID_VALUE";
      break;
    case (GL_INVALID_OPERATION):
      msg += " GL_INVALID_OPERATION";
      break;
    case (GL_STACK_OVERFLOW):
      msg += " GL_STACK_OVERFLOW";
      break;
    case (GL_STACK_UNDERFLOW):
      msg += " GL_STACK_UNDERFLOW";
      break;
    case (GL_OUT_OF_MEMORY):
      msg += " GL_OUT_OF_MEMORY";
      break;
    case (GL_TABLE_TOO_LARGE):
      msg += " GL_TABLE_TOO_LARGE";
      break;
    default:
      msg += " unknown error";
    }
    if (err != GL_NO_ERROR)
      std::cout << "glGetError " << msg << " " << err << std::endl;
  }
 
private:
  Texture::sptr texture_;
  Texture::sptr textureCol_;
  GLuint vao_;
  GLuint vbo_[2];
  Program::sptr  prog_;
} ;


/// FLTK window holding the spectrum display
class SqliteSpecBrowser : public Fl_Double_Window {
public:
      typedef boost::posix_time::ptime ptime;
  SqliteSpecBrowser(int w, int h)
    : Fl_Double_Window(w, h, "Spectrum Display")
    , menu_bar_   (  0,    0,   w,     20)
    , input_      (  0, h-30,   w,     30, "label")
    , browserTop_ (  0,   20,   w,     70)
    , browserLeft_(  0,   90, w-300, h-120)
    , glPlot_     (300,   90, w-300, h-120)
      // , disp_(20, 40, w-40, h-40, "Display") 
  {
    menu_bar_.add("File/Open", FL_CTRL+'o', static_cb_menuBar, this);
    menu_bar_.add("File/Quit", FL_CTRL+'q', static_cb_menuBar, this);

    const int widths[] = { 40, 200, 200, 50, 50, 80, 80, 0 };
    browserTop_.column_widths(widths);
    browserTop_.add("specId\ttMin\ttMax\tnSpec\tfMin\tfMax\tdf");
    browserTop_.column_char('\t');
    browserTop_.type(FL_MULTI_BROWSER);
    browserTop_.callback(static_cb_browserTop, this);

    browserLeft_.callback(static_cb_browserLeft, this);
    end();
  }
  virtual ~SqliteSpecBrowser() {}


protected:
  static void static_cb_browserTop(Fl_Widget *, void *data) {
    SqliteSpecBrowser *w = (SqliteSpecBrowser*)(data);
    std::cout << "selected: " << w->browserTop_.value() << std::endl;

    w->proc(w->browserTop_.value()-1);
  }
  static void static_cb_browserLeft(Fl_Widget *, void *data) {
    SqliteSpecBrowser *w = (SqliteSpecBrowser*)(data);
    std::cout << "selected: " << w->browserLeft_.value() << std::endl;

    w->displaySignal(w->browserLeft_.value());
  }
  static void static_cb_menuBar(Fl_Widget *, void *data) {
    SqliteSpecBrowser *w = (SqliteSpecBrowser*)(data);
    std::cout << "callback " << data << " " << w << std::endl;
    w->callback();
  }

  void displaySignal(int signalId) {
    std::cout << "displaySignal: " << browserLeft_.text(signalId) << " " << vs_[signalId-1].first << " " << vs_[signalId-1].second << std::endl;

    const std::pair<size_t, size_t> interval(vs_[signalId-1]);

    const int width = interval.second - interval.first;

    const int length = fv_[interval.first].second.size();


    size_t dy = 4+width;
    size_t imgWidth  = 16*60;
    size_t imgHeight = dy*(2+length/imgWidth);

    GLubyte *img = new GLubyte[imgHeight*imgWidth*1];

    for (size_t i=0; i<imgHeight; ++i)
      for (size_t j=0; j<imgWidth; ++j)
        img[i*imgWidth + j] = GLubyte(0);

    for (int is=interval.first; is<interval.second+1; ++is) {
      int j = 0;
      int i = is - interval.first;
      for (int js=0; js<length; ++js) {
        img[i*imgWidth + j++] = GLubyte(255 * clamp((fv_[is].second[js] + 72.0)/(-55.0 + 72.0)));

        if (j == imgWidth) {
          j  = 0;
          i += dy;
        }
      }
    }
    
    glActiveTexture(GL_TEXTURE0);
    glPlot_.texture()->bind();
    
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, imgWidth, imgHeight, 0, GL_RED, GL_UNSIGNED_BYTE, img);

    delete[] img;
    
    glGenerateMipmap(GL_TEXTURE_2D);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);     

    glPlot_.redraw();

  }

  static float clamp(float x) {
    return std::max(std::min(x,1.0f),0.0f);
  }

  void callback() {
    std::cout << "callback_" << this << std::endl;
    std::cout << "callback_" << " " << menu_bar_.mvalue() << std::endl;
    static char picked[1024] = { 0 };
    if (menu_bar_.item_pathname(picked, sizeof(picked)-1, NULL) == 0) {
      std::cout << "callback_" << std::endl;
      printf("CALLBACK: You picked '%s'\n", picked);

      if ( strcmp(picked, "File/Quit") == 0 ) {
        exit(0);
      }

      if ( strcmp(picked, "File/Open") == 0 ) {
        Fl_Native_File_Chooser fnfc;
        fnfc.title("Pick a file");
        fnfc.type(Fl_Native_File_Chooser::BROWSE_FILE);
        fnfc.filter("*.db");
        fnfc.directory("./");
        
        switch ( fnfc.show() ) {
        case -1: printf("ERROR: %s\n", fnfc.errmsg());    break;  // ERROR
        case  1: printf("CANCEL\n");                      break;  // CANCEL
        default: printf("PICKED: %s\n", fnfc.filename());        // FILE CHOSEN
          db_ = db::sqlite3::connection::make(fnfc.filename());

          db::sqlite3::statement stmt(db_, "SELECT specId,tMin,tMax,nSpec,fMin,fMax,df from SpecInfo;");
          
          while (stmt.step()) {
            const int  specId(stmt.get_column<int>   (0));
            const ptime  tMin(stmt.get_column<ptime> (1));
            const ptime  tMax(stmt.get_column<ptime> (2));
            const int   nSpec(stmt.get_column<int>   (3));
            const double fMin(stmt.get_column<double>(4));
            const double fMax(stmt.get_column<double>(5));
            const double   df(stmt.get_column<double>(6));

            const std::string line(str(boost::format("%5d\t%s\t%s\t%6d\t%7.0f\t%7.0f\t%5.3f") % specId % tMin % tMax % nSpec % fMin % fMax % df));
            browserTop_.add(line.c_str());            
          }

          break;              
        }
      }
    }
  }

  void proc(int specId) {
    db::sqlite3::statement stmt(db_, str(boost::format("SELECT tMin,tMax,nSpec,fMin,fMax,df from SpecInfo WHERE specId==%d;") % specId));
    std::cout << "A" << std::endl;
    stmt.step();
    std::cout << "B" << std::endl;
//     const ptime  tMin(stmt.get_column<ptime> (0));
//     const ptime  tMax(stmt.get_column<ptime> (1));
//     const int   nSpec(stmt.get_column<int>   (2));
    const double fMin(stmt.get_column<double>(3));
    const double fMax(stmt.get_column<double>(4));
    const double   df(stmt.get_column<double>(5));
    std::cout << "C" << std::endl;

    fv_ = fv_type(fMin, fMax, size_t((fMax-fMin)/df+0.5), std::deque<double >());
    std::cout << "D" << std::endl;

    Filter::Cascaded<frequency_vector<double> > filter;
    filter.add(Filter::LowPass<frequency_vector<double> >::make(1.0, 15));
    
    ptime t0(boost::posix_time::not_a_date_time);

    std::cout << "E" << std::endl;
    db::sqlite3::statement stmt2(db_, str(boost::format("SELECT t,pMin,pMax,s from SpecData WHERE specId==%d;") % specId));
    std::cout << "F" << std::endl;
    while (stmt2.step()) {
      const ptime  t     = stmt2.get_column<ptime> (0);
      const double pMin  = stmt2.get_column<double>(1);
      const double pMax  = stmt2.get_column<double>(2);
      const size_t n     = stmt2.column_bytes(3);
      const unsigned char *s = static_cast<const unsigned char*>(stmt2.column_blob(3));
      std::cout << "t= " << t << " " << pMin << " " << pMax << " " << n << std::endl;
      
      if (t0 == boost::posix_time::not_a_date_time)
        t0 = t;
      
      std::cout << t0 << " " << t << " " << t-t0 << std::endl;
      
      std::vector<double> vs(n);
      
      frequency_vector<double> ps(fMin, fMax, size_t((fMax-fMin)/df+0.5));
      
      for (fv_type::iterator i(fv_.begin()), iend(fv_.end()); i!=iend; ++i) {
        const ssize_t index(std::distance(fv_.begin(), i));
        i->second.push_back(pMin+(pMax-pMin)*s[index]/255.0);
        ps[index].second = pMin+(pMax-pMin)*s[index]/255.0;
      }

      if (filter.x().empty())
        filter.init(t, ps);
      else
        filter.update(t, ps);
      
      if (t-t0 > boost::posix_time::minutes(4)) {
        t0 = t;
        
        const frequency_vector<double>& xf(filter.x());
        
        std::vector<double> spec_filtered(ps.size(), 0);
        for (size_t i=0, n=ps.size(); i<n; ++i)
          spec_filtered[i] = xf[i].second;
        
        std::vector<size_t> b(ps.size(), 1);
        const double threshold_db(2);
        const unsigned poly_degree(2);
        
        const size_t m(15*4); // number of fit intervals
        std::vector<double> indices(m+1, 0);
        for (size_t i(0); i<m; ++i)
          indices[i] = (i*n)/m;
        
        indices[m] = n-1;
        indices[0] = std::min(indices[1],   indices[0]+size_t(0.005*n));
        indices[m] = std::max(indices[m-1], indices[m]-size_t(0.005*n));
        
        polynomial_interval_fit p(poly_degree, indices);
        std::vector<double> ts(n, 0);
        for (size_t i(0); i<n; ++i)
          ts[i] = i;
        
        for (size_t l(0); l<100; ++l) {
          if (!p.fit(ts, spec_filtered, b)) {
            std::cerr << "fit failed" << std::endl;
          }
          size_t nchanged(0);
          for (size_t i(0); i<n; ++i) {
            const std::pair<double,double> vf(p.eval(i));
            const bool c(spec_filtered[i]-vf.first > threshold_db);
            nchanged += (c==b[i]);
            b[i] = !c;
          }
          if (0 == nchanged)
            break;
        }
        
        // b[i] == false -> signal
        // b[i] == true  -> no signal
        
        // (1) group signals according to amplitude

        vs_.clear();
        std::pair<size_t, size_t> interval  = std::make_pair(0,0);
        for (size_t i(1); i<n; ++i) {
          const std::pair<double,double> vf(p.eval(i));
          std::cout << "F: " << i << " " << fMin+df*i << " " << b[i] << " " << spec_filtered[i] << " " << vf.first << std::endl;
          if (!b[i-1] && b[i]) {
            interval.second = i-1;
            interval.first = (interval.first == 0) ? i-1 : interval.first;
            vs_.push_back(interval);
            std::cout << "S: " << vs_.size()-1 << " " << interval.first << " " << interval.second << " "
                      << fMin+df*interval.first << " " << fMin+df*interval.second << std::endl;
            
            const std::string line(str(boost::format("%7.0f %7.0f")
                                       % (fMin+df*interval.first)
                                       % (fMin+df*interval.second)));
            browserLeft_.add(line.c_str(), this);
            interval.first = interval.second = 0;
          }
          if (!b[i-1] && !b[i]) {
            interval.first = (interval.first == 0) ? i-1 : interval.first;
          }
          if (b[i])
            continue;
        }
        //           for (size_t j=0,m=vs_.size(); j<m; ++j) {
        //             std::cout << "S: " << j << " " << vs_[j].first << " " << vs_[j].second << " "
        //                       << fMin+df*vs_[j].first << " " << fMin+df*vs_[j].second << std::endl;
        
        // //             const double f = fMin+df*i;
        // //             const std::pair<double,double> vf(p.eval(i));
        // //             std::cout << i << " " << f << " " << b[i] << " " << spec_filtered[i] << " " << vf.first << std::endl;;
        //           }
        
        
        
        //           for (int j=0; j<n; ++j) {
        //             vs_[j] = pMin+(pMax-pMin)*s[j]/255.0;
        //             const double f = fMin+df*j;
        //             std::cout << j << " " << f << " " << vs_[j] << std::endl;;
        //             if (f>331.8e3 && f< 333.2e3)
        //               std::cout << j << " " << f << " " << vs_[j] << std::endl;;
        //           }
        break;
      }
    }

  }


private:

  // GUI
  Fl_Menu_Bar menu_bar_;
  Fl_Input    input_;
  Fl_Select_Browser  browserTop_;
  Fl_Select_Browser  browserLeft_;
  OpenGlPlot  glPlot_;
  // DB
  db::sqlite3::connection::sptr db_;

  typedef frequency_vector<std::deque<double> > fv_type;
  fv_type fv_;
  std::vector<std::pair<size_t, size_t> > vs_; // signals

} ;


int main(int argc, char* argv[])
{
  Fl::visual(FL_RGB | FL_OPENGL3);
  
  SqliteSpecBrowser w(1200,600);
  w.show();
  return (Fl::run());  

  LOGGER_INIT("./Log", "test_spec_sqlite3");
  try {
    

    std::cout << "file: " << argv[1] << std::endl;

    db::sqlite3::connection::sptr db =
      db::sqlite3::connection::make(argv[1]);

    db::sqlite3::statement stmt(db, "SELECT specId,tMin,tMax,nSpec,fMin,fMax,df from SpecInfo;");

    while (stmt.step()) {
      
      std::cout << "column_count " << stmt.column_count() << std::endl;
      for (int i=0; i<stmt.column_count(); ++i) {
        std::cout << i << " " << stmt.column_type(i) << std::endl;
      }
      std::cout << stmt.get_column<int>(0) << std::endl;
      typedef boost::posix_time::ptime ptime;

      const int specId(stmt.get_column<int>(0));
      const ptime tMin(stmt.get_column<ptime>(1));
      const ptime tMax(stmt.get_column<ptime>(2));
//       const int nSpec(stmt.get_column<int>(3));
      const double fMin(stmt.get_column<double>(4));
      const double fMax(stmt.get_column<double>(5));
      const double df(stmt.get_column<double>(6));
      std::cout << tMin << " " << tMax << " " << fMin << " " << fMax << std::endl;

      std::cout << stmt.get_column<int>(3) << std::endl;
      std::cout << stmt.get_column<double>(4) << std::endl;
      std::cout << stmt.get_column<double>(5) << std::endl;
      std::cout << stmt.get_column<double>(6) << std::endl;

      typedef frequency_vector<std::deque<double> > fv_type;
      fv_type fv(fMin, fMax, size_t((fMax-fMin)/df+0.5), std::deque<double >());

      Filter::Cascaded<frequency_vector<double> > filter;
      filter.add(Filter::LowPass<frequency_vector<double> >::make(1.0, 15));

      ptime t0(boost::posix_time::not_a_date_time);

      db::sqlite3::statement stmt2(db, str(boost::format("SELECT t,pMin,pMax,s from SpecData WHERE specId==%d;") % specId));
      while (stmt2.step()) {
        const ptime  t     = stmt2.get_column<ptime> (0);
        const double pMin  = stmt2.get_column<double>(1);
        const double pMax  = stmt2.get_column<double>(2);
        const size_t n     = stmt2.column_bytes(3);
        const unsigned char *s = static_cast<const unsigned char*>(stmt2.column_blob(3));
        std::cout << "t= " << t << " " << pMin << " " << pMax << " " << n << std::endl;

        if (t0 == boost::posix_time::not_a_date_time)
          t0 = t;

        std::cout << t0 << " " << t << " " << t-t0 << std::endl;

        std::vector<double> vs(n);

        frequency_vector<double> ps(fMin, fMax, size_t((fMax-fMin)/df+0.5));

        for (fv_type::iterator i(fv.begin()), iend(fv.end()); i!=iend; ++i) {
          const ssize_t index(std::distance(fv.begin(), i));
          i->second.push_back(pMin+(pMax-pMin)*s[index]/255.0);
          ps[index].second = pMin+(pMax-pMin)*s[index]/255.0;
        }

        if (filter.x().empty())
          filter.init(t, ps);
        else
          filter.update(t, ps);

        if (t-t0 > boost::posix_time::minutes(3)) {
          t0 = t;
          
          const frequency_vector<double>& xf(filter.x());
          
          std::vector<double> spec_filtered(ps.size(), 0);
          for (size_t i=0, n=ps.size(); i<n; ++i)
            spec_filtered[i] = xf[i].second;
          
          std::vector<size_t> b(ps.size(), 1);
          const double threshold_db(2);
          const unsigned poly_degree(2);
          
          const size_t m(15*4); // number of fit intervals
          std::vector<double> indices(m+1, 0);
          for (size_t i(0); i<m; ++i)
            indices[i] = (i*n)/m;
          
          indices[m] = n-1;
          indices[0] = std::min(indices[1],   indices[0]+size_t(0.005*n));
          indices[m] = std::max(indices[m-1], indices[m]-size_t(0.005*n));
          
          polynomial_interval_fit p(poly_degree, indices);
          std::vector<double> ts(n, 0);
          for (size_t i(0); i<n; ++i)
            ts[i] = i;
          
          for (size_t l(0); l<100; ++l) {
            if (!p.fit(ts, spec_filtered, b)) {
              std::cerr << "fit failed" << std::endl;
            }
            size_t nchanged(0);
            for (size_t i(0); i<n; ++i) {
              const std::pair<double,double> vf(p.eval(i));
              const bool c(spec_filtered[i]-vf.first > threshold_db);
              nchanged += (c==b[i]);
              b[i] = !c;
            }
            if (0 == nchanged)
              break;
          }

          // b[i] == false -> signal
          // b[i] == true  -> no signal

          // (1) group signals according to amplitude
          
          std::vector<std::pair<size_t, size_t> > vs; // signals
          std::pair<size_t, size_t> interval  = std::make_pair(0,0);
          for (size_t i(1); i<n; ++i) {
            const std::pair<double,double> vf(p.eval(i));
            std::cout << "F: " << i << " " << fMin+df*i << " " << b[i] << " " << spec_filtered[i] << " " << vf.first << std::endl;
            if (!b[i-1] && b[i]) {
              interval.second = i-1;
              interval.first = (interval.first == 0) ? i-1 : interval.first;
              vs.push_back(interval);
              std::cout << "S: " << vs.size()-1 << " " << interval.first << " " << interval.second << " "
                        << fMin+df*interval.first << " " << fMin+df*interval.second << std::endl;
              
              interval.first = interval.second = 0;
            }
            if (!b[i-1] && !b[i]) {
              interval.first = (interval.first == 0) ? i-1 : interval.first;
            }
            if (b[i])
              continue;
          }
//           for (size_t j=0,m=vs.size(); j<m; ++j) {
//             std::cout << "S: " << j << " " << vs[j].first << " " << vs[j].second << " "
//                       << fMin+df*vs[j].first << " " << fMin+df*vs[j].second << std::endl;

// //             const double f = fMin+df*i;
// //             const std::pair<double,double> vf(p.eval(i));
// //             std::cout << i << " " << f << " " << b[i] << " " << spec_filtered[i] << " " << vf.first << std::endl;;
//           }


          
//           for (int j=0; j<n; ++j) {
//             vs[j] = pMin+(pMax-pMin)*s[j]/255.0;
//             const double f = fMin+df*j;
//             std::cout << j << " " << f << " " << vs[j] << std::endl;;
//             if (f>331.8e3 && f< 333.2e3)
//               std::cout << j << " " << f << " " << vs[j] << std::endl;;
//           }
          
          
        }
      }
    }    

  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    LOG_ERROR(e.what()); 
    return 1;
  }
  return 0;
}
