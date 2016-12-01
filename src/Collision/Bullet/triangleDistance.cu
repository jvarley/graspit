  __global__ void computeTriangleStuff(float *dPoints, float* dResults, int num_points, float x, float y, float z, float qx, float qy, float qz, float qw) {
    int idx = ThreadIdx.x + blockIdx.x * blockDim.x;
    
    if(idx * 12 < num_points) {
        dResults[idx * 4] = 0;
        dResults[idx * 4 + 1] = 0;//etc.
        
        //All the triangle computation goes
        
        
      }
  }