/* -*- opencl -*- */

inline float2 complex_mul(float2 c1, float2 c2) {
  float a = c1.x, b = c1.y;
  float c = c2.x, d = c2.y;
  float k1 = a*(c + d), k2 = d*(a + b), k3 = c*(b - a);
  return (float2)(k1-k2, k1+k3);
}

__kernel void convolve(__global float2 *out,
		       __global const float2* in,
		       __global const float2* h,
		       const int n,
		       const int d,
		       const int shift,
		       const float norm)
{

  const int j = get_global_id(0);
  const int nd = n/d;
  int i;
  float2 sum = 0;
#if 1
  for (i=0; i<d; i++) {
    sum += complex_mul(in[(n+j+i*nd+shift)%n], h[j+i*nd]);
  }
#else
  for (i=0; i<n; i+=nd) {
    sum += complex_mul(in[(n+j+i+shift)%n], h[j+i]);
  }
#endif
  out[j] = norm*sum;

#if 0
  printf("global,local_size(0) %5d,%5d global,local_id(0)  %5d,%5d (%f,%f) (%f,%f) (%d,%d,%d,%f) sum=(%f,%f)\n",
	 get_global_size(0), get_local_size(0),
	 get_global_id(0), get_local_id(0),
	 out[0].x, out[0].y,
	 h[0].x, h[0].y,
	 n,d,shift,norm,
	 sum.x,sum.y
	 );
#endif
}
