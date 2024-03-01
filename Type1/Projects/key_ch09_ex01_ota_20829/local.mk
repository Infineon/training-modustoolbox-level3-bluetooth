################################################################################
# \file local.mk
# \version 1.0
#
# \brief
# Makefile containing the COMPONENTS and DEFINES across all the Targets.
# To be used only for the ota-update repo's test build.
#
################################################################################
# \copyright
# Copyright 2018-2022 Cypress Semiconductor Corporation
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

####################################################################################
# Common COMPONENTS and DEFINES
####################################################################################

# Common components across all TARGETS
COMPONENTS+=PSOC6HAL FREERTOS

# TODO: Look through the DEFINES to see if these are already defined, don't need to do it twice
DEFINES+= CY_RETARGET_IO_CONVERT_LF_TO_CRLF CY_RTOS_AWARE

ifeq ($(OTA_BT_ONLY),1)
    COMPONENTS+=WICED_BLE
ifneq ($(filter $(OTA_PLATFORM),PSOC_062_1M PSOC_063_1M),)
    COMPONENTS+=CM0P_BLESS_OTA
    DISABLE_COMPONENTS+=CM0P_BLESS

    # needed for test/COMPONENT_OTA_BLUETOOTH/app_bt_cfg.c
    # to use .hci_transport = CYBT_HCI_IPC
    DEFINES+=BLESS_PORTING_LAYER

    $(info: ***Components included = WICED_BLE, SOFTFP, BTSS-IPC )
endif
endif

ifneq ($(filter $(OTA_PLATFORM),PSOC_062_2M PSOC_062_512K PSOC_064_2M XMC7200),)
    COMPONENTS+=MBEDTLS SECURE_SOCKETS
ifeq ($(OTA_BT_SUPPORT),1)
    COMPONENTS+=WICED_BLE HCI-UART
endif
endif

ifneq ($(filter 1,$(OTA_HTTP_SUPPORT) $(OTA_MQTT_SUPPORT)),)
    COMPONENTS+=MBEDTLS SECURE_SOCKETS
ifeq ($(OTA_PLATFORM),XMC7200)
	COMPONENTS+=ECM PSOC6HAL
    DEFINES+=CYBSP_ETHERNET_CAPABLE
else
    DEFINES+=CYBSP_WIFI_CAPABLE
endif

ifneq ($(OTA_BT_ONLY),1)
    COMPONENTS+=LWIP
endif
ifneq ($(OTA_BT_SUPPORT),1)
    DISABLE_COMPONENTS+=WICED_BLE HCI-UART
endif
endif

ifeq ($(OTA_PLATFORM),CYW20829)
    # Misc for 20829
    CORE?=CM33
    APPTYPE = flash
    CY_LCS  = NORMAL_NO_SECURE
    SMIF_CRYPTO_CONFIG ?= NONE
    COMPONENTS+=SOFTFP BTSS-IPC
endif

ifeq ($(OTA_PLATFORM),XMC7200)
    # Misc for XMC7200
    APP_CORE?=CM7
    APPTYPE = flash
endif


ifeq ($(ACTUAL_TARGET), CYW920829M2EVK-02)
    COMPONENTS+=BTFW-TX10 CYW20829B0
    DISABLE_COMPONENTS=FIRMWARE-TX10
endif

##################################################################################
# This OTA test Application IGNORE
##################################################################################

# Always ignore this directory - it conflicts with sprintf in lib_nano.a
CY_IGNORE+=$(SEARCH_command-console)/source/iperf
CY_IGNORE+=$(SEARCH_command-console)/source/iperf_utility

# Always ignore this directory - it conflicts with our ota-update test app console initialization
CY_IGNORE+=$(SEARCH_command-console)/source/bluetooth_utility

# # Always ignore this directory. Command_console brings in bluetooth-freertos, but we do not want it
CY_IGNORE+=$(SEARCH_bluetooth-freertos)

####################################################################################
# This OTA Test Application includes / Defines
####################################################################################

INCLUDES+=test/configs\
          test/configs/COMPONENT_$(CORE)

ifeq ($(CY_BUILD_LOCATION),)
    # NOTE: This needs to be different from "build" in our library as GitLab scripts use that directory
    CY_BUILD_LOCATION:=./bld
endif

ifeq ($(TEST_SWAP_SETUP),1)
    DEFINES+=TEST_SWAP_SETUP=1
endif

ifeq ($(TEST_SWAP_REVERT),1)
    DEFINES+=TEST_SWAP_REVERT=1
endif

# If not defined, set to 0 (default)
ifeq ($(OTA_TEST_APP_VERSION_IN_TAR),)
    OTA_TEST_APP_VERSION_IN_TAR=0
endif
