# This makefile contains settings needed for OTA using the anycloud-ota libray

#IMPORTANT - This file must be included in the applicaton Makefile AFTER the "Advanced Configuration" section.
#            Specifically, it must be included after the COMPONENTS, INCLUDES, DEFINES, LDFLAGS, and LINKER_SCRIPT
#            variables are set because they are set in the application Makefile without using +=.

# Application version. This must be updated for the OTA process to recognize that newer FW is availble in the secondary slot
APP_VERSION_MAJOR?=1
APP_VERSION_MINOR?=1
APP_VERSION_BUILD?=0

# Include the file that has the path to the anyclould-ota library
-include ./libs/mtb.mk

# Create a variable for the target name that has underscores instead of dashes
# This is needed because of the directory name used in the anycloud-ota library
TARGET_UNDERSCORE=$(subst -,_,$(TARGET))

# Core processor
CORE?=CM4

# Set to 1 to add OTA defines, sources, and libraries (must be used with MCUBoot)
OTA_SUPPORT=1

# Set to 1 to add OTA external flash support
# Set to 0 for internal flash
# Make sure MCUboot has the same configuration
OTA_USE_EXTERNAL_FLASH?=1

# Set the AnyCloud OTA library to use Bluetooth as the OTA transport
COMPONENTS+=OTA_BLUETOOTH

###########################################################################
#
# OTA Support
#
ifeq ($(OTA_SUPPORT),1)
    # OTA / MCUBoot defines

    # These libs are only needed for WiFi transports so we will exclude them from the build
    CY_IGNORE+=$(SEARCH_aws-iot-device-sdk-embedded-C)
    CY_IGNORE+=$(SEARCH_aws-iot-device-sdk-port)
    CY_IGNORE+=$(SEARCH_http-client)
    CY_IGNORE+=$(SEARCH_mqtt)
    CY_IGNORE+=$(SEARCH_secure-sockets)
    CY_IGNORE+=$(SEARCH_wifi-connection-manager)
    CY_IGNORE+=$(SEARCH_lwip)
    CY_IGNORE+=$(SEARCH_mbedtls)
    CY_IGNORE+=$(SEARCH_cy-mbedtls-acceleration)
    CY_IGNORE+=$(SEARCH_wifi-host-driver)
    CY_IGNORE+=$(SEARCH_wifi-mw-core)
    CY_IGNORE+=$(SEARCH_whd-bsp-integration)
    CY_IGNORE+=$(SEARCH_wpa3-external-supplicant)

    # IMPORTANT NOTE: These defines are also used in the building of MCUBOOT
    #                 they must EXACTLY match the values added to
    #                 mcuboot/boot/cypress/MCUBootApp/MCUBootApp.mk
    #
    # Must be a multiple of 1024 (must leave __vectors on a 1k boundary)
    MCUBOOT_HEADER_SIZE=0x400
    ifeq ($(OTA_USE_EXTERNAL_FLASH),1)
        MCUBOOT_MAX_IMG_SECTORS=32
        # SCRATCH SIZE is the size of FLASH set aside for the SWAP of Primary and Secondary Slots
        # Please see mcuboot documentation for more information
        CY_BOOT_SCRATCH_SIZE=0x0004000
        # Boot loader size defines for mcuboot & app are different, but value is the same
        MCUBOOT_BOOTLOADER_SIZE=0x00018000
        CY_BOOT_BOOTLOADER_SIZE=$(MCUBOOT_BOOTLOADER_SIZE)
        # Primary Slot Currently follows Bootloader sequentially
        CY_BOOT_PRIMARY_1_START=0x00018000
        CY_BOOT_PRIMARY_1_SIZE=0x001C0000
        # offset from start of external FLASH 18000000
        CY_BOOT_SECONDARY_1_START=0x00000000
        CY_BOOT_SECONDARY_1_SIZE=0x001C0000
        CY_FLASH_ERASE_VALUE=0xFF
    else
        MCUBOOT_MAX_IMG_SECTORS=32
        CY_BOOT_SCRATCH_SIZE=0x0004000
        # Boot loader size defines for mcuboot & app are different, but value is the same
        MCUBOOT_BOOTLOADER_SIZE=0x00018000
        CY_BOOT_BOOTLOADER_SIZE=$(MCUBOOT_BOOTLOADER_SIZE)
        # Primary Slot Currently follows Bootloader sequentially
        CY_BOOT_PRIMARY_1_START=0x00018000
        CY_BOOT_PRIMARY_1_SIZE=0x000EE000
        CY_BOOT_SECONDARY_1_SIZE=0x000EE000
        CY_BOOT_PRIMARY_2_SIZE=0x01000
        CY_BOOT_SECONDARY_2_START=0x001E0000
        CY_FLASH_ERASE_VALUE=0x00
    endif

    # Additional / custom linker flags.
    # This needs to be before finding LINKER_SCRIPT_WILDCARD as we need the extension defined
    ifeq ($(TOOLCHAIN),GCC_ARM)
    CY_ELF_TO_HEX=$(CY_CROSSPATH)/bin/arm-none-eabi-objcopy
    CY_ELF_TO_HEX_OPTIONS="-O ihex"
    CY_ELF_TO_HEX_FILE_ORDER="elf_first"
    CY_TOOLCHAIN=GCC
    CY_TOOLCHAIN_LS_EXT=ld
    LDFLAGS+="-Wl,--defsym,MCUBOOT_HEADER_SIZE=$(MCUBOOT_HEADER_SIZE),--defsym,MCUBOOT_BOOTLOADER_SIZE=$(MCUBOOT_BOOTLOADER_SIZE),--defsym,CY_BOOT_PRIMARY_1_SIZE=$(CY_BOOT_PRIMARY_1_SIZE)"
    else
    ifeq ($(TOOLCHAIN),IAR)
    CY_ELF_TO_HEX=$(CY_CROSSPATH)/bin/ielftool
    CY_ELF_TO_HEX_OPTIONS="--ihex"
    CY_ELF_TO_HEX_FILE_ORDER="elf_first"
    CY_TOOLCHAIN=$(TOOLCHAIN)
    CY_TOOLCHAIN_LS_EXT=icf
    LDFLAGS+=--config_def MCUBOOT_HEADER_SIZE=$(MCUBOOT_HEADER_SIZE) --config_def MCUBOOT_BOOTLOADER_SIZE=$(MCUBOOT_BOOTLOADER_SIZE) --config_def CY_BOOT_PRIMARY_1_SIZE=$(CY_BOOT_PRIMARY_1_SIZE)
    else
    ifeq ($(TOOLCHAIN),ARM)
    CY_ELF_TO_HEX=$(CY_CROSSPATH)/bin/fromelf.exe
    CY_ELF_TO_HEX_OPTIONS="--i32 --output"
    CY_ELF_TO_HEX_FILE_ORDER="hex_first"
    CY_TOOLCHAIN=GCC
    CY_TOOLCHAIN_LS_EXT=sct
    LDFLAGS+=--pd=-DMCUBOOT_HEADER_SIZE=$(MCUBOOT_HEADER_SIZE) --pd=-DMCUBOOT_BOOTLOADER_SIZE=$(MCUBOOT_BOOTLOADER_SIZE) --pd=-DCY_BOOT_PRIMARY_1_SIZE=$(CY_BOOT_PRIMARY_1_SIZE)
    else
    LDFLAGS+=
    endif #ARM
    endif #IAR
    endif #GCC_ARM

    # Linker Script
    LINKER_SCRIPT_WILDCARD:= $(SEARCH_anycloud-ota)/$(TARGET_UNDERSCORE)/COMPONENT_$(CORE)/TOOLCHAIN_$(TOOLCHAIN)/ota/*_ota_int.$(CY_TOOLCHAIN_LS_EXT)
    LINKER_SCRIPT:=$(wildcard $(LINKER_SCRIPT_WILDCARD))

    # MCUBoot flash support location
    MCUBOOT_DIR= $(SEARCH_anycloud-ota)/source/mcuboot

    # build location
    BUILD_LOCATION=./build

    # output directory for use in the sign_script.bash
    OUTPUT_FILE_PATH=$(BUILD_LOCATION)/$(TARGET)/$(CONFIG)

    DEFINES+=OTA_SUPPORT=1 \
        MCUBOOT_HEADER_SIZE=$(MCUBOOT_HEADER_SIZE) \
        MCUBOOT_MAX_IMG_SECTORS=$(MCUBOOT_MAX_IMG_SECTORS) \
        CY_BOOT_SCRATCH_SIZE=$(CY_BOOT_SCRATCH_SIZE) \
        MCUBOOT_IMAGE_NUMBER=1\
        MCUBOOT_BOOTLOADER_SIZE=$(MCUBOOT_BOOTLOADER_SIZE) \
        CY_BOOT_BOOTLOADER_SIZE=$(CY_BOOT_BOOTLOADER_SIZE) \
        CY_BOOT_PRIMARY_1_START=$(CY_BOOT_PRIMARY_1_START) \
        CY_BOOT_PRIMARY_1_SIZE=$(CY_BOOT_PRIMARY_1_SIZE) \
        CY_BOOT_SECONDARY_1_START=$(CY_BOOT_SECONDARY_1_START) \
        CY_BOOT_SECONDARY_1_SIZE=$(CY_BOOT_SECONDARY_1_SIZE) \
        CY_BOOT_PRIMARY_2_SIZE=$(CY_BOOT_PRIMARY_2_SIZE) \
        CY_BOOT_SECONDARY_2_START=$(CY_BOOT_SECONDARY_2_START) \
        CY_FLASH_ERASE_VALUE=$(CY_FLASH_ERASE_VALUE)\
        APP_VERSION_MAJOR=$(APP_VERSION_MAJOR)\
        APP_VERSION_MINOR=$(APP_VERSION_MINOR)\
        APP_VERSION_BUILD=$(APP_VERSION_BUILD)

    ifeq ($(OTA_USE_EXTERNAL_FLASH),1)
        DEFINES+=CY_BOOT_USE_EXTERNAL_FLASH=1
    endif

    CY_HEX_TO_BIN="$(CY_COMPILER_GCC_ARM_DIR)/bin/arm-none-eabi-objcopy"
    CY_BUILD_VERSION=$(APP_VERSION_MAJOR).$(APP_VERSION_MINOR).$(APP_VERSION_BUILD)

    # For non-secure OTA:
    #
    # Use "create" for IMGTOOL_COMMAND_ARG below to create an unsigned image.
    #
    # MCUBoot must also be modified to skip checking the signature
    #   Comment out and re-build MCUBootApp
    #   <mcuboot>/boot/cypress/MCUBootApp/config/mcuboot_config/mcuboot_config.h
    #   line 37, 38, 77
    # 37: //#define MCUBOOT_SIGN_EC256
    # 38: //#define NUM_ECC_BYTES (256 / 8)   // P-256 curve size in bytes, rnok: to make compilable
    # 77: //#define MCUBOOT_VALIDATE_PRIMARY_SLOT

    # For secure OTA:
    #
    # Use "sign" for IMGTOOL_COMMAND_ARG below to create a signed image.
    # The MCUBOOT_KEY_DIR and MCUBOOT_KEY_FILE values are examples only. They MUST be replaced with
    # the customer's key path/file for a production design.
    # 
    # MCUBoot must also be modified to check the signature
    #   Uncomment and re-build MCUBootApp
    #   <mcuboot>/boot/cypress/MCUBootApp/config/mcuboot_config/mcuboot_config.h
    #   line 37, 38, 77
    # 37: #define MCUBOOT_SIGN_EC256
    # 38: #define NUM_ECC_BYTES (256 / 8)   // P-256 curve size in bytes, rnok: to make compilable
    # 77: #define MCUBOOT_VALIDATE_PRIMARY_SLOT

    ifneq ($(TARGET),MULTI_CM0_CM4)
        # signing scripts and keys from MCUBoot
        # Defaults for 062 non-secure boards
        SIGN_SCRIPT_FILE_PATH=$(SEARCH_anycloud-ota)/scripts/sign_script.bash
        IMGTOOL_SCRIPT_NAME=imgtool_v1.7.0/imgtool.py
        MCUBOOT_SCRIPT_FILE_DIR=$(MCUBOOT_DIR)/scripts
        MCUBOOT_KEY_DIR=$(MCUBOOT_DIR)/keys
        MCUBOOT_KEY_FILE=$(MCUBOOT_KEY_DIR)/cypress-test-ec-p256.pem
        IMGTOOL_COMMAND_ARG=create
        CY_SIGNING_KEY_ARG=" "

        POSTBUILD=$(SIGN_SCRIPT_FILE_PATH) $(OUTPUT_FILE_PATH) $(APPNAME) $(CY_PYTHON_PATH)\
                  $(CY_ELF_TO_HEX) $(CY_ELF_TO_HEX_OPTIONS) $(CY_ELF_TO_HEX_FILE_ORDER)\
                  $(MCUBOOT_SCRIPT_FILE_DIR) $(IMGTOOL_SCRIPT_NAME) $(IMGTOOL_COMMAND_ARG) $(CY_FLASH_ERASE_VALUE) $(MCUBOOT_HEADER_SIZE)\
                  $(MCUBOOT_MAX_IMG_SECTORS) $(CY_BUILD_VERSION) $(CY_BOOT_PRIMARY_1_START) $(CY_BOOT_PRIMARY_1_SIZE)\
                  $(CY_HEX_TO_BIN) $(CY_SIGNING_KEY_ARG)
    else
        # preparing for possible multi-image use in future
        SIGN_SCRIPT_FILE_PATH=$(SEARCH_anycloud-ota)/scripts/sign_tar.bash
        MCUBOOT_KEY_DIR=$(CY_MCUBOOT_SCRIPT_FILE_DIR)/keys
        CY_SIGN_SCRIPT_FILE_PATH=$(SEARCH_anycloud-ota)/scripts/sign_tar.bash
        MCUBOOT_KEY_FILE=$(CY_MCUBOOT_KEY_DIR)/cypress-test-ec-p256.pem
        IMGTOOL_COMMAND_ARG=sign
        CY_SIGNING_KEY_ARG="-k $(MCUBOOT_KEY_FILE)"

        POSTBUILD=$(CY_SIGN_SCRIPT_FILE_PATH) $(CY_OUTPUT_FILE_PATH) $(CY_BUILD) $(CY_OBJ_COPY)\
                $(CY_MCUBOOT_SCRIPT_FILE_DIR) $(IMGTOOL_SCRIPT_NAME) $(IMGTOOL_COMMAND_ARG) $(CY_FLASH_ERASE_VALUE) $(MCUBOOT_HEADER_SIZE)\
                $(CY_BUILD_VERSION) $(CY_BOOT_PRIMARY_1_START) $(CY_BOOT_PRIMARY_1_SIZE)\
                $(CY_BOOT_PRIMARY_2_SIZE) $(CY_BOOT_SECONDARY_1_START)\
                $(MCUBOOT_KEY_DIR) $(CY_SIGNING_KEY_ARG)
    endif

endif # OTA Support


