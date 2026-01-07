#include <cuda_runtime.h>
#include <iostream>
#include <Eigen/Dense>


__global__ void transformPoints(float* d_points, float* d_results, float* d_T_wv, float* d_T_vl, float* d_T_lw, int numPoints, int ring_num) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < numPoints) {
        
        float X_l = d_points[idx*3];
        float Y_l = d_points[idx*3 + 1];
        float Z_l = d_points[idx*3 + 2];

        float T_wv[16] = &d_T_wv[idx / 520 * 16];

        // Lidar to Vechile
        float X_v = d_T_vl[0] * X_l + d_T_vl[1] * Y_l + d_T_vl[2] * Z_l + d_T_vl[3];
        float Y_v = d_T_vl[4] * X_l + d_T_vl[5] * Y_l + d_T_vl[6] * Z_l + d_T_vl[7];
        float Z_v = d_T_vl[8] * X_l + d_T_vl[9] * Y_l + d_T_vl[10] * Z_l + d_T_vl[11];

        // Vechicle to world
        float X_w = T_wv[0] * X_v + T_wv[1] * Y_v + T_wv[2] * Z_v + T_wv[3];
        float Y_w = T_wv[4] * X_v + T_wv[5] * Y_v + T_wv[6] * Z_v + T_wv[7];
        float Z_w = T_wv[8] * X_v + T_wv[9] * Y_v + T_wv[10] * Z_v + T_wv[11];

        // World to l
        d_results[idx*3] = d_T_lw[0] * X_w + d_T_lw[1] * Y_w + d_T_lw[2] * Z_w + d_T_lw[3];
        d_results[idx*3 + 1] = d_T_lw[4] * X_w + d_T_lw[5] * Y_w + d_T_lw[6] * Z_w + d_T_lw[7];
        d_results[idx*3 + 2] = d_T_lw[8] * X_w + d_T_lw[9] * Y_w + d_T_lw[10] * Z_w + d_T_lw[11];
    }
}




extern "C" void transformPointsGPU(float* h_points, float* h_results, float* h_T_wv, float* h_T_vl,  float* h_T_lw, int numPoints, int timestamp_size, int ring_num) {
    float* d_points;
    float* d_results;
    float* d_T_wv;
    float* d_T_vl;
    float* d_T_lw;


    size_t pointsSize = numPoints * 3 * sizeof(float);
    size_t matricesSize1 = timestamp_size * 16 * sizeof(float); 
    size_t matricesSize2 = 16 * sizeof(float); 

    cudaMalloc(&d_points, pointsSize);
    cudaMalloc(&d_results, pointsSize);
    cudaMalloc(&d_T_wv, matricesSize1);
    cudaMalloc(&d_T_vl, matricesSize2);
    cudaMalloc(&h_T_lw, matricesSize2);

    cudaMemcpy(d_points, h_points, pointsSize, cudaMemcpyHostToDevice);
    cudaMemcpy(d_T_wv, h_T_wv, matricesSize1, cudaMemcpyHostToDevice);
    cudaMemcpy(d_T_vl, h_T_vl, matricesSize2, cudaMemcpyHostToDevice);
    cudaMemcpy(d_T_lw, h_T_lw, matricesSize2, cudaMemcpyHostToDevice);

    int blockSize = 256;
    int gridSize = (numPoints + blockSize - 1) / blockSize;

    transformPoints<<<gridSize, blockSize>>>(d_points, d_results, d_T_wv, d_T_vl, d_T_lw, numPoints, ring_num);

    cudaDeviceSynchronize();
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "CUDA Error: " << cudaGetErrorString(err) << std::endl;
    }

    cudaMemcpy(h_results, d_results, pointsSize, cudaMemcpyDeviceToHost);

    cudaFree(d_points);
    cudaFree(d_results);
    cudaFree(d_T_wv);
    cudaFree(d_T_vl);
    cudaFree(d_T_lw);
}

