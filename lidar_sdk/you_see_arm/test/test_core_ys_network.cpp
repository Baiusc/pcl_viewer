/*
 * @Author       : kaihong.yang kaihong.yang@vitalchem.com
 * @Date         : 2024-06-19 23:43:54
 * @LastEditors  : kaihong.yang kaihong.yang@vitalchem.com
 * @LastEditTime : 2024-06-24 05:11:34
 * @FilePath     : /you_see_arm/test/test_core_ys_network.cpp
 * @Description  :
 * Copyright (c) 2024 by Vitalchem, All Rights Reserved.
 */

#include <gtest/gtest.h>

extern "C" {
#include "../core/ys_core_network.h"
}

TEST(ys_core_network, get_ip_via_interface)
{
	uint8_t dest[8];
	ASSERT_EQ(0, get_ip_via_interface("eth0", dest));
	ASSERT_EQ(-1, get_ip_via_interface("_test", dest));
}

TEST(ys_core_network, get_mask_via_interface)
{
	uint8_t dest[8];
	ASSERT_EQ(0, get_mask_via_interface("eth0", dest));
	ASSERT_EQ(-1, get_mask_via_interface("_test", dest));
}

TEST(ys_core_network, get_gateway_via_interface)
{
	uint8_t dest[8];
	ASSERT_EQ(0, get_gateway_via_interface("eth0", dest));
	ASSERT_EQ(-1, get_gateway_via_interface("_test", dest));
}

TEST(ys_core_network, get_mac_via_interface)
{
	uint8_t dest[8];
	ASSERT_EQ(0, get_mac_via_interface("eth0", dest));
	ASSERT_EQ(-1, get_mac_via_interface("_test", dest));
}
