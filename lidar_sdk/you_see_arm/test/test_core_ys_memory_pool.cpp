/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-11-01 03:27:31
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2024-11-04 07:04:46
 * @FilePath     : /you_see_arm/test/test_core_ys_memory_pool.cpp
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#include <gtest/gtest.h>

extern "C" {
#include "../core/ys_memory_pool.h"
}

TEST(ys_memory_pool, CreateMemoryPool)
{
	memory_pool_t *pool;

	set_block_size(16);
	pool = create_memory_pool(16);

	ASSERT_TRUE(pool != NULL);

	destroy_memory_pool(pool);
}

TEST(ys_memory_pool, CheckMemoryPoolSize)
{
	memory_pool_t *pool;
	int size = 16;
	int block_size = 16;

	set_block_size(block_size);
	pool = create_memory_pool(size);

	ASSERT_TRUE(pool != NULL);

	ASSERT_EQ(size, pool->size);

	destroy_memory_pool(pool);
}
