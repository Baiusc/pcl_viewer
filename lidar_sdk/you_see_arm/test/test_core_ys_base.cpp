/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-06-18 02:52:14
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2024-11-01 03:16:23
 * @FilePath     : /you_see_arm/test/test_core_ys_base.cpp
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#include <gtest/gtest.h>

extern "C" {
#include "../core/ys_base.h"
}

TEST(ys_base, transform_uint8_to_float)
{
	uint8_t data[4] = { 0x00, 0x00, 0x40, 0x41 };
	ASSERT_EQ(12.000000, transform_uint8_to_float(data));
}

TEST(ys_base, transform_uint32_to_float)
{
	uint32_t data = 0x41400000;
	ASSERT_EQ(12.000000, transform_uint32_to_float(data));
}

TEST(ys_base, transform_float_to_uint8)
{
	uint8_t data[4];
	transform_float_to_uint8(12.0, data);

	uint8_t expected_data[4] = { 0x00, 0x00, 0x40, 0x41 };
	ASSERT_EQ(memcmp(data, expected_data, sizeof(data)), 0);

	{
		uint8_t data[4];
		transform_float_to_uint8(-12.123, data);

		uint8_t expected_data[4] = { 0xcf, 0xf7, 0x41, 0xc1 };
		ASSERT_EQ(memcmp(data, expected_data, sizeof(data)), 0);
	}
}

TEST(ys_base, transform_uint32_to_uint8)
{
	uint8_t data[4];
	uint32_t value = 12;
	transform_uint32_to_uint8(value, data);

	uint8_t expected_data[4] = { 0x00, 0x00, 0x00, 0x0c };
	ASSERT_EQ(memcmp(data, expected_data, sizeof(data)), 0);
}

TEST(ys_base, transform_int16_to_uint8)
{
	uint8_t data[2];
	short value = 12;
	transform_int16_to_uint8(value, data);

	uint8_t expected_data[2] = { 0x00, 0x0c };
	ASSERT_EQ(memcmp(data, expected_data, sizeof(data)), 0);
}

// 测试正常分配内存
TEST(ys_base, NormalAllocation)
{
	int *ptr = (int *)Malloc(sizeof(int));
	ASSERT_TRUE(ptr != NULL); // 使用ASSERT_TRUE来避免类型比较问题

	// 验证内存可用
	*ptr = 42;
	EXPECT_EQ(*ptr, 42);
	free(ptr);
}

// 测试分配0字节
TEST(ys_base, ZeroSize)
{
	void *ptr = Malloc(0);
	// C标准允许malloc(0)返回NULL或非NULL指针
	free(ptr); // free(NULL)在C中是安全的
}

// 测试分配大内存
TEST(ys_base, LargeAllocation)
{
	// 尝试分配一个非常大的内存块
	const size_t large_size = SIZE_MAX / 2;
	void *ptr = Malloc(large_size);

	// ptr预期为NULL，因为请求的内存太大
	if (ptr == NULL) {
		// 这里可以添加日志验证的代码
	} else {
		free(ptr);
	}
}

// 测试多次分配和释放
TEST(ys_base, MultipleAllocations)
{
	const int num_allocations = 1000;
	void *ptrs[1000];
	int i;

	// 多次分配
	for (i = 0; i < num_allocations; i++) {
		ptrs[i] = Malloc(100);
		ASSERT_TRUE(ptrs[i] != NULL); // 使用ASSERT_TRUE来避免类型比较问题
	}

	// 释放所有分配的内存
	for (i = 0; i < num_allocations; i++) {
		free(ptrs[i]);
	}
}

// 测试内存对齐
TEST(ys_base, MemoryAlignment)
{
	// 测试不同大小的内存分配是否正确对齐
	void *ptr1 = Malloc(1);
	void *ptr2 = Malloc(2);
	void *ptr4 = Malloc(4);
	void *ptr8 = Malloc(8);

	// 验证地址对齐
	EXPECT_EQ((uintptr_t)ptr4 % 4, 0);
	EXPECT_EQ((uintptr_t)ptr8 % 8, 0);

	free(ptr1);
	free(ptr2);
	free(ptr4);
	free(ptr8);
}

TEST(ys_base, FreeNormalCase)
{
	// 正常情况：分配内存并释放
	int *ptr = (int *)Malloc(sizeof(int));
	ASSERT_TRUE(ptr != NULL);
	*ptr = 42;

	Free((void **)&ptr);
	EXPECT_TRUE(ptr == NULL); // 验证指针被设置为NULL
}

TEST(ys_base, FreeNullPointer)
{
	// 测试传入NULL指针
	void *ptr = NULL;
	Free((void **)&ptr); // 不应该崩溃
	EXPECT_TRUE(ptr == NULL);
}

TEST(ys_base, FreeNullPointerPointer)
{
	// 测试传入NULL的指针
	void **ptr = NULL;
	Free(ptr); // 不应该崩溃
}

TEST(ys_base, FreeMultipleTimes)
{
	// 测试多次释放同一个指针
	int *ptr = (int *)Malloc(sizeof(int));
	ASSERT_TRUE(ptr != NULL);

	Free((void **)&ptr);
	EXPECT_TRUE(ptr == NULL);

	// 再次释放已经为NULL的指针应该是安全的
	Free((void **)&ptr);
	EXPECT_TRUE(ptr == NULL);
}

TEST(ys_base, FreeMallocLoop)
{
	// 测试循环分配释放
	int *ptr = NULL;
	for (int i = 0; i < 100; i++) {
		ptr = (int *)Malloc(sizeof(int));
		ASSERT_TRUE(ptr != NULL);
		*ptr = i;
		Free((void **)&ptr);
		EXPECT_TRUE(ptr == NULL);
	}
}

TEST(ys_base, crc16_checkout)
{
	const uint8_t data[7] = { 0xaa, 0x01, 0x00, 0x02, 0x00, 0x00 };
	ASSERT_EQ(0x8411, crc16_checkout(data, 6));
	ASSERT_EQ(0x1163, crc16_checkout(data, 7));
}

TEST(ys_base, crc32_checkout)
{
	const uint8_t data[4] = { 0x02, 0x00, 0x00, 0x01 };
	ASSERT_EQ(0xFC4A2701, crc32_checkout(data, 4));
}

TEST(ys_base, combine_uint8_to_uint16)
{
	ASSERT_EQ(0x0000, combine_uint8_to_uint16(0x00, 0x00));
	ASSERT_EQ(0x0001, combine_uint8_to_uint16(0x00, 0x01));
	ASSERT_EQ(0x0100, combine_uint8_to_uint16(0x01, 0x00));
	ASSERT_EQ(0x00FF, combine_uint8_to_uint16(0x00, 0xFF));
	ASSERT_EQ(0xFF00, combine_uint8_to_uint16(0xFF, 0x00));
	ASSERT_EQ(0x0010, combine_uint8_to_uint16(0x00, 0x10));
}

TEST(ys_base, combine_uint8_to_uint32)
{
	ASSERT_EQ(0x00000000, combine_uint8_to_uint32(0x00, 0x00, 0x00, 0x00));
	ASSERT_EQ(0x00000001, combine_uint8_to_uint32(0x00, 0x00, 0x00, 0x01));
	ASSERT_EQ(0x00000010, combine_uint8_to_uint32(0x00, 0x00, 0x00, 0x10));
	ASSERT_EQ(0x00000100, combine_uint8_to_uint32(0x00, 0x00, 0x01, 0x00));
	ASSERT_EQ(0x00001000, combine_uint8_to_uint32(0x00, 0x00, 0x10, 0x00));
	ASSERT_EQ(0x00010000, combine_uint8_to_uint32(0x00, 0x01, 0x00, 0x00));
	ASSERT_EQ(0x00100000, combine_uint8_to_uint32(0x00, 0x10, 0x00, 0x00));
	ASSERT_EQ(0x01000000, combine_uint8_to_uint32(0x01, 0x00, 0x00, 0x00));
	ASSERT_EQ(0x10000000, combine_uint8_to_uint32(0x10, 0x00, 0x00, 0x00));
}

// 获取一个时间戳
TEST(ys_base, TimestampBasic)
{
	long long ts = get_milliseconds_timestamp();

	// 验证时间戳是否在合理范围内
	// 使用2020年1月1日的时间戳作为下限
	long long lower_bound = 1577836800000LL; // 2020-01-01 00:00:00
	// 使用2030年为上限
	long long upper_bound = 1893456000000LL; // 2030-01-01 00:00:00

	EXPECT_GT(ts, lower_bound);
	EXPECT_LT(ts, upper_bound);
}

// 测试连续调用时间戳是否递增
TEST(ys_base, TimestampIncreasing)
{
	long long ts1 = get_milliseconds_timestamp();

	// 稍微等待一小段时间
	usleep(1000); // 等待1毫秒

	long long ts2 = get_milliseconds_timestamp();
	EXPECT_GT(ts2, ts1);
}

// 测试时间戳的精度
TEST(ys_base, TimestampPrecision)
{
	long long ts1 = get_milliseconds_timestamp();
	long long ts2 = get_milliseconds_timestamp();

	// 两次快速调用的时间差应该很小（通常小于10ms）
	long long diff = ts2 - ts1;
	EXPECT_GE(diff, 0);
	EXPECT_LT(diff, 10);
}

// 测试多次调用的稳定性
TEST(ys_base, TimestampStability)
{
	const int ITERATIONS = 1000;
	long long prev_ts = get_milliseconds_timestamp();

	for (int i = 0; i < ITERATIONS; i++) {
		long long current_ts = get_milliseconds_timestamp();
		EXPECT_GE(current_ts, prev_ts);
		prev_ts = current_ts;
	}
}

// 测试时间戳格式（13位）
TEST(ys_base, TimestampFormat)
{
	long long ts = get_milliseconds_timestamp();

	// 计算位数
	long long temp = ts;
	int digits = 0;
	while (temp > 0) {
		temp /= 10;
		digits++;
	}

	// 验证是13位的时间戳
	EXPECT_EQ(digits, 13);
}

TEST(ys_base, is_power_of_two)
{
	ASSERT_EQ(true, is_power_of_two(4));
	ASSERT_EQ(false, is_power_of_two(3));
	ASSERT_EQ(true, is_power_of_two(8));
	ASSERT_EQ(true, is_power_of_two(1));
}

TEST(ys_base, roundup_pow_of_two)
{
	ASSERT_EQ(256, roundup_pow_of_two(255));
	ASSERT_EQ(256, roundup_pow_of_two(250));
	ASSERT_EQ(32, roundup_pow_of_two(25));
}
