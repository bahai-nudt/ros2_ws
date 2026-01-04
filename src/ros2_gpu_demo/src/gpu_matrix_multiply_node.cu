#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <cuda_runtime.h>
#include <iostream>
#include <vector>

using namespace std;

// CUDA 核心函数：矩阵乘法
__global__ void matrix_multiply_kernel(float* A, float* B, float* C, int N) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    if (idx < N && idy < N) {
        float sum = 0;
        for (int k = 0; k < N; k++) {
            sum += A[idy * N + k] * B[k * N + idx];
        }
        C[idy * N + idx] = sum;
    }
}

// ROS2 节点，执行 GPU 计算并发布结果
class GpuMatrixMultiplyNode : public rclcpp::Node {
public:
    GpuMatrixMultiplyNode() : Node("gpu_matrix_multiply_node") {
        // 创建一个发布者
        publisher_ = this->create_publisher<std_msgs::msg::String>("gpu_result", 10);

        // 定时器，每秒运行一次
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&GpuMatrixMultiplyNode::compute_and_publish, this)
        );
    }

private:
    void compute_and_publish() {
        const int N = 3;  // 矩阵大小

        // 在主机上分配内存
        std::vector<float> h_A(N * N, 1.0f);  // 矩阵 A，所有元素都初始化为 1
        std::vector<float> h_B(N * N, 2.0f);  // 矩阵 B，所有元素都初始化为 2
        std::vector<float> h_C(N * N, 0.0f);  // 结果矩阵 C

        // 在设备上分配内存
        float *d_A, *d_B, *d_C;
        cudaMalloc(&d_A, N * N * sizeof(float));
        cudaMalloc(&d_B, N * N * sizeof(float));
        cudaMalloc(&d_C, N * N * sizeof(float));

        // 将矩阵数据从主机拷贝到设备
        cudaMemcpy(d_A, h_A.data(), N * N * sizeof(float), cudaMemcpyHostToDevice);
        cudaMemcpy(d_B, h_B.data(), N * N * sizeof(float), cudaMemcpyHostToDevice);

        // 定义 CUDA 网格和块大小
        dim3 blockSize(16, 16);
        dim3 gridSize((N + blockSize.x - 1) / blockSize.x, (N + blockSize.y - 1) / blockSize.y);

        // 执行矩阵乘法
        matrix_multiply_kernel<<<gridSize, blockSize>>>(d_A, d_B, d_C, N);

        // 检查 CUDA 错误
        cudaError_t error = cudaGetLastError();
        if (error != cudaSuccess) {
            std::cerr << "CUDA error: " << cudaGetErrorString(error) << std::endl;
        }

        // 将结果从设备拷贝回主机
        cudaMemcpy(h_C.data(), d_C, N * N * sizeof(float), cudaMemcpyDeviceToHost);

        // 释放设备内存
        cudaFree(d_A);
        cudaFree(d_B);
        cudaFree(d_C);

        // 打印结果矩阵
        std::string result = "Matrix C: \n";
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                result += std::to_string(h_C[i * N + j]) + " ";
            }
            result += "\n";
        }

        // 发布结果
        auto message = std_msgs::msg::String();
        message.data = result;
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpuMatrixMultiplyNode>());
    rclcpp::shutdown();
    return 0;
}

