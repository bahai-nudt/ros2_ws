#include <cuda_runtime.h>
#include <iostream>



// 四元数内积（点积）
__device__ float dot(const float* q0, const float* q1) {
    return q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3];
}

// 四元数归一化
__device__ void normalize(float* q) {
    float norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
}

// 四元数乘标量
__device__ void scale(float* q, float scalar, float* result) {
    result[0] = q[0] * scalar;
    result[1] = q[1] * scalar;
    result[2] = q[2] * scalar;
    result[3] = q[3] * scalar;
}

// 四元数加法
__device__ void add(const float* q0, const float* q1, float* result) {
    result[0] = q0[0] + q1[0];
    result[1] = q0[1] + q1[1];
    result[2] = q0[2] + q1[2];
    result[3] = q0[3] + q1[3];
}




__device__ void slerp(float* q0, float* q1, float* result, float t) {

    normalize(q0);
    normalize(q1);

    float dot_val = dot(q0, q1);



    if (dot_val < 0.0f) {
        q1[0] = -q1[0];
        q1[1] = -q1[1];
        q1[2] = -q1[2];
        q1[3] = -q1[3];
        dot_val = -dot_val;
    }

    const float THRESHOLD = 0.9995f;
    float theta_0 = acos(dot_val);  // 计算夹角
    float theta = theta_0 * t;

    // 对于内积接近1的情况，使用线性插值来避免精度问题



    if (dot_val > THRESHOLD) {
        // 线性插值
        float result_quat[4];
        scale(q0, (1.0f - t), result_quat);
        float quat1_scaled[4];
        scale(q1, t, quat1_scaled);
        add(result_quat, quat1_scaled, result);
        normalize(result);
    } else {
        float sin_theta_0 = sin(theta_0);
        float sin_theta = sin(theta);
        float sin_theta_1 = sin(theta_0 - theta);

        float result_quat[4];
        scale(q0, (sin_theta_1 / sin_theta_0), result_quat);
        float quat1_scaled[4];
        scale(q1, (sin_theta / sin_theta_0), quat1_scaled);
        add(result_quat, quat1_scaled, result);
    }
}


__global__ void transformPoints(float* d_points, float* d_results, float* d_T_wv, float* d_T_vl, float* d_T_lw, int numPoints, int ring_num) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < numPoints) {
        
        float X_l = d_points[idx*3];
        float Y_l = d_points[idx*3 + 1];
        float Z_l = d_points[idx*3 + 2];

        float T_wv[12] = {d_T_wv[idx / 520 * 12], d_T_wv[idx / 520 * 12 + 1], d_T_wv[idx / 520 * 12 + 2], d_T_wv[idx / 520 * 12 + 3],
                          d_T_wv[idx / 520 * 12 + 4], d_T_wv[idx / 520 * 12 + 5], d_T_wv[idx / 520 * 12 + 6], d_T_wv[idx / 520 * 12 + 7],
                          d_T_wv[idx / 520 * 12 + 8], d_T_wv[idx / 520 * 12 + 9], d_T_wv[idx / 520 * 12 + 10], d_T_wv[idx / 520 * 12 + 11]};

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

__global__ void transformPoints_OFFLINE(float* d_points, float* d_results, float* d_T_vl, float* d_T_lw, float* d_ins_pose, int numPoints) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < numPoints) {
        
        float X_l = d_points[idx*4];
        float Y_l = d_points[idx*4 + 1];
        float Z_l = d_points[idx*4 + 2];
        float timestamp = d_points[idx*4 + 3];




        float pose_before[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        float pose_after[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // find before pose and after pose
        for (int i = 0; i < 15; i++) {


            // if (idx == 0) {
                // printf("pts[0]=%f %f %f\n", pose_before[0], pose_before[1], pose_before[2]);
                // printf("pts[0]=%f\n", timestamp);
                // printf("pts[1]=%i\n", i);
                // printf("pts[0]=%f\n", d_ins_pose[i*8 + 1]);
                // printf("pts[0]=%f\n", d_ins_pose[i*8 + 2]);
                // printf("pts[0]=%f\n", d_ins_pose[i*8 + 3]);
                // printf("pts[0]=%f\n", d_ins_pose[i*8 + 4]);
                // printf("pts[0]=%f\n", d_ins_pose[i*8 + 5]);
                // printf("pts[0]=%f\n", d_ins_pose[i*8 + 6]);
                // printf("pts[2]=%f\n", d_ins_pose[i*8 + 7]);
            // }

            if (timestamp < d_ins_pose[i*8 + 7]) {


                pose_before[0] = d_ins_pose[(i-1)*8]; pose_before[1] = d_ins_pose[(i-1)*8 + 1]; pose_before[2] = d_ins_pose[(i-1)*8 + 2]; pose_before[3] = d_ins_pose[(i-1)*8 + 3];
                pose_before[4] = d_ins_pose[(i-1)*8 + 4]; pose_before[5] = d_ins_pose[(i-1)*8 + 5]; pose_before[6] = d_ins_pose[(i-1)*8 + 6]; pose_before[7] = d_ins_pose[(i-1)*8 + 7];

                pose_after[0] = d_ins_pose[i*8]; pose_after[1] = d_ins_pose[i*8 + 1]; pose_after[2] = d_ins_pose[i*8 + 2]; pose_after[3] = d_ins_pose[i*8 + 3];
                pose_after[4] = d_ins_pose[i*8 + 4]; pose_after[5] = d_ins_pose[i*8 + 5]; pose_after[6] = d_ins_pose[i*8 + 6]; pose_after[7] = d_ins_pose[i*8 + 7];

                break;
            }
        }

        float result[4] = {1.0, 0.0, 0.0, 0.0};

        float t = (timestamp - pose_before[7]) / (pose_after[7] - pose_before[7]);
        slerp(pose_before + 3, pose_after + 3, result, t);

        float w = result[0];
        float x = result[1];
        float y = result[2];
        float z = result[3];




        float R[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        float position[3] = {0.0, 0.0, 0.0};

        R[0] = 1.0f - 2.0f * (y * y + z * z);
        R[1] = 2.0f * (x * y - z * w);
        R[2] = 2.0f * (x * z + y * w);
        R[3] = 2.0f * (x * y + z * w);
        R[4] = 1.0f - 2.0f * (x * x + z * z);
        R[5] = 2.0f * (y * z - x * w);
        R[6] = 2.0f * (x * z - y * w);
        R[7] = 2.0f * (y * z + x * w);
        R[8] = 1.0f - 2.0f * (x * x + y * y);

        position[0] = (1-t)*pose_before[0] + t*pose_after[0];
        position[1] = (1-t)*pose_before[1] + t*pose_after[1];
        position[2] = (1-t)*pose_before[2] + t*pose_after[2];





        // Lidar to Vechile
        float X_v = d_T_vl[0] * X_l + d_T_vl[1] * Y_l + d_T_vl[2] * Z_l + d_T_vl[3];
        float Y_v = d_T_vl[4] * X_l + d_T_vl[5] * Y_l + d_T_vl[6] * Z_l + d_T_vl[7];
        float Z_v = d_T_vl[8] * X_l + d_T_vl[9] * Y_l + d_T_vl[10] * Z_l + d_T_vl[11];





        // Vechicle to world
        float X_w = R[0] * X_v + R[1] * Y_v + R[2] * Z_v + position[0];
        float Y_w = R[3] * X_v + R[4] * Y_v + R[5] * Z_v + position[1];
        float Z_w = R[6] * X_v + R[7] * Y_v + R[8] * Z_v + position[2];




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
    size_t matricesSize1 = timestamp_size * 12 * sizeof(float); 
    size_t matricesSize2 = 12 * sizeof(float); 

    cudaMalloc(&d_points, pointsSize);
    cudaMalloc(&d_results, pointsSize);
    cudaMalloc(&d_T_wv, matricesSize1);
    cudaMalloc(&d_T_vl, matricesSize2);
    cudaMalloc(&d_T_lw, matricesSize2);

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


extern "C" void transformPointsGPU_OFFLINE(float* h_points, float* h_results, float* h_T_vl, float* h_T_lw, float* h_ins_pose, int numPoints) {
    float* d_points;
    float* d_results;
    float* d_T_vl;
    float* d_T_lw;
    float* d_ins_pose;

    size_t pointsSize = numPoints * 4 * sizeof(float);
    size_t resultSize = numPoints * 3 * sizeof(float);

    size_t matricesSize = 12 * sizeof(float); 
    size_t poseSize = 15 * 8 * sizeof(float);

    cudaMalloc(&d_points, pointsSize);
    cudaMalloc(&d_results, resultSize);
    cudaMalloc(&d_T_vl, matricesSize);
    cudaMalloc(&d_T_lw, matricesSize);
    cudaMalloc(&d_ins_pose, poseSize);

    cudaMemcpy(d_points, h_points, pointsSize, cudaMemcpyHostToDevice);
    cudaMemcpy(d_results, h_results, resultSize, cudaMemcpyHostToDevice);
    cudaMemcpy(d_T_vl, h_T_vl, matricesSize, cudaMemcpyHostToDevice);
    cudaMemcpy(d_T_lw, h_T_lw, matricesSize, cudaMemcpyHostToDevice);
    cudaMemcpy(d_ins_pose, h_ins_pose, poseSize, cudaMemcpyHostToDevice);

    int blockSize = 256;
    int gridSize = (numPoints + blockSize - 1) / blockSize;

    transformPoints_OFFLINE<<<gridSize, blockSize>>>(d_points, d_results, d_T_vl, d_T_lw, d_ins_pose, numPoints);

    cudaDeviceSynchronize();
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        std::cerr << "CUDA Error: " << cudaGetErrorString(err) << std::endl;
    }

    cudaMemcpy(h_results, d_results, resultSize, cudaMemcpyDeviceToHost);

    cudaFree(d_points);
    cudaFree(d_results);
    cudaFree(d_T_vl);
    cudaFree(d_T_lw);
    cudaFree(d_ins_pose);
}