#include "opencv2/core/cuda/common.hpp"
#include "opencv2/core/cuda/utility.hpp"
#include <cuda/mat_norm.hpp>
#include <helper_cuda.h>
#include <Utils.hpp>

using namespace cv;
using namespace cv::cuda;
using namespace cv::cuda::device;

namespace ORB_SLAM2 { namespace cuda {

MatNormGPU::MatNormGPU() {
    // GpuMat should be already stored in memory
}

MatNormGPU::~MatNormGPU() {
}

__global__
void kernel_get_mat_pixel (uint8_t * src, int w)
{
    // assuming that we resized it to CV_32F so the channel number is 1
    subtract_val = src[ (w*step) + (w)];
}

void MatNormGPU::setSubtractValue(cv::cuda::GpuMat _img, int w)
{
    kernel_get_mat_pixel<<1, 1>>(_img.data, w);
}

__global__
void kernel_subtract_pixel_from_mat (uint8_t * src, int MaxRows, int MaxCols, int step)
{
    unsigned int row = blockIdx.x * blockDim.x + threadIdx.x; //Row number
    unsigned int col = blockIdx.y * blockDim.y + threadIdx.y; //Column number
    //unsigned int ch = blockIdx.z * blockDim.z + threadIdx.z; //Channel 0

    if (row<MaxRows && col<MaxCols) {
        int idx = row * step + col; // maxChannels is 1 and ch is 0
        src[idx] = src[idx] - subtract_val;
    }
}

void MatNormGPU::subtract_pixel_from_mat (cv::cuda::GpuMat _img)
{
    dim3 tpb(16, 16);
    dim3 bpg(((_img.cols + 15) / 16), ((_img.rows + 15)/ 16));
    kernel_subtract_pixel_from_mat<<bpg, tpb>> (_img.data,_img.rows, _img.cols, _img.step);
}

} }
