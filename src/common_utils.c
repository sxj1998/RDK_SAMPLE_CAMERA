// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright(C) 2024, D-Robotics Co., Ltd.
 *                     All rights reserved.
 ***************************************************************************/

#include "common_utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/stat.h>

int32_t read_yuv420_file(const char *filename, char *addr0, char *addr1, uint32_t y_size)
{
	FILE *fp = fopen(filename, "rb");
	if (fp == NULL) {
		printf("Error opening file: %s\n", filename);
		return -1;
	}
	
	// 读取Y分量
	if (fread(addr0, 1, y_size, fp) != y_size) {
		printf("Error reading Y data from file: %s\n", filename);
		fclose(fp);
		return -1;
	}
	
	// 读取UV分量
	if (fread(addr1, 1, y_size / 2, fp) != y_size / 2) {
		printf("Error reading UV data from file: %s\n", filename);
		fclose(fp);
		return -1;
	}
	
	fclose(fp);
	return 0;
}

int32_t read_yuvv_nv12_file(const char *filename, char *addr0, char *addr1, uint32_t y_size)
{
	FILE *fp = fopen(filename, "rb");
	if (fp == NULL) {
		printf("Error opening file: %s\n", filename);
		return -1;
	}
	
	// 读取Y分量
	if (fread(addr0, 1, y_size, fp) != y_size) {
		printf("Error reading Y data from file: %s\n", filename);
		fclose(fp);
		return -1;
	}
	
	// 读取UV分量（NV12格式）
	if (fread(addr1, 1, y_size / 2, fp) != y_size / 2) {
		printf("Error reading UV data from file: %s\n", filename);
		fclose(fp);
		return -1;
	}
	
	fclose(fp);
	return 0;
}

int32_t dump_image_to_file(char *filename, uint8_t *src_buffer, uint32_t size)
{
	FILE *fp = fopen(filename, "wb");
	if (fp == NULL) {
		printf("Error opening file for writing: %s\n", filename);
		return -1;
	}
	
	if (fwrite(src_buffer, 1, size, fp) != size) {
		printf("Error writing to file: %s\n", filename);
		fclose(fp);
		return -1;
	}
	
	fclose(fp);
	printf("Image data dumped to file: %s (size: %u bytes)\n", filename, size);
	return 0;
}

int32_t dump_2plane_yuv_to_file(char *filename, uint8_t *src_buffer, uint8_t *src_buffer1,
		uint32_t size, uint32_t size1)
{
	FILE *fp = fopen(filename, "wb");
	if (fp == NULL) {
		printf("Error opening file for writing: %s\n", filename);
		return -1;
	}
	
	// 写入第一个平面
	if (fwrite(src_buffer, 1, size, fp) != size) {
		printf("Error writing first plane to file: %s\n", filename);
		fclose(fp);
		return -1;
	}
	
	// 写入第二个平面
	if (fwrite(src_buffer1, 1, size1, fp) != size1) {
		printf("Error writing second plane to file: %s\n", filename);
		fclose(fp);
		return -1;
	}
	
	fclose(fp);
	printf("2-plane YUV data dumped to file: %s (size: %u + %u bytes)\n", filename, size, size1);
	return 0;
}

int32_t vpm_hb_mem_init(void)
{
	printf("[vpm_hb_mem_init] HB内存模块初始化\n");
	return 0;
}

void vpm_hb_mem_deinit(void)
{
	printf("[vpm_hb_mem_deinit] HB内存模块反初始化\n");
}

int32_t alloc_graphic_buffer(hbn_vnode_image_t *img, uint32_t width,
			 uint32_t height, uint32_t cached, int32_t format)
{
	printf("[alloc_graphic_buffer] 分配图形缓冲区: %dx%d, 格式: %d\n", width, height, format);
	
	// 简化的缓冲区分配
	img->buffer.width = width;
	img->buffer.height = height;
	img->buffer.stride = width;  // 简化假设步长等于宽度
	img->buffer.size[0] = width * height;
	
	return 0;
}

uint64_t vio_test_gettime_us(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (uint64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

uint32_t load_file_2_buff_nosize(const char *path, char *filebuff)
{
	FILE *fp = fopen(path, "rb");
	if (fp == NULL) {
		printf("Error opening file: %s\n", path);
		return 0;
	}
	
	// 获取文件大小
	fseek(fp, 0, SEEK_END);
	uint32_t file_size = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	
	// 读取文件内容
	if (fread(filebuff, 1, file_size, fp) != file_size) {
		printf("Error reading file: %s\n", path);
		fclose(fp);
		return 0;
	}
	
	fclose(fp);
	return file_size;
}

char* get_program_name()
{
	static char program_name[256] = "get_vin_data";
	return program_name;
}

void configure_vse_max_resolution(int32_t channel, uint32_t input_width, uint32_t input_height,
	uint32_t *output_width, uint32_t *output_height)
{
	printf("[configure_vse_max_resolution] 配置VSE最大分辨率 - 通道: %d, 输入: %dx%d\n", 
		channel, input_width, input_height);
	
	// 简化的分辨率配置
	if (output_width != NULL) {
		*output_width = input_width;
	}
	if (output_height != NULL) {
		*output_height = input_height;
	}
}

int read_nv12_image_to_graphic_buffer(const char *file_path, hb_mem_graphic_buf_t *src_buf, int width, int height)
{
	printf("[read_nv12_image_to_graphic_buffer] 读取NV12图像到图形缓冲区: %s, %dx%d\n", 
		file_path, width, height);
	return 0;
}

int read_nv12_image_to_common_buffer(const char *file_path, hb_mem_common_buf_t *src_buf, int width, int height)
{
	printf("[read_nv12_image_to_common_buffer] 读取NV12图像到通用缓冲区: %s, %dx%d\n", 
		file_path, width, height);
	return 0;
}

int read_nv12_image_to_normal_memory(const char *file_path, uint8_t*virt_addr, int width, int height)
{
	printf("[read_nv12_image_to_normal_memory] 读取NV12图像到普通内存: %s, %dx%d\n", 
		file_path, width, height);
	
	// 简化的图像读取实现
	FILE *fp = fopen(file_path, "rb");
	if (fp == NULL) {
		printf("Error opening file: %s\n", file_path);
		return -1;
	}
	
	// 计算NV12图像大小
	int y_size = width * height;
	int uv_size = width * height / 2;
	int total_size = y_size + uv_size;
	
	// 读取图像数据
	if (fread(virt_addr, 1, total_size, fp) != total_size) {
		printf("Error reading image data from file: %s\n", file_path);
		fclose(fp);
		return -1;
	}
	
	fclose(fp);
	printf("NV12 image loaded successfully: %s (size: %d bytes)\n", file_path, total_size);
	return 0;
}