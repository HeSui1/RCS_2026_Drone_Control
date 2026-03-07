cmake_minimum_required(VERSION 3.22)

# =========================================================
# 1. 项目与基础配置
# =========================================================
# 设置项目名称
set(CMAKE_PROJECT_NAME COD_H7_Template)
project(${CMAKE_PROJECT_NAME} C ASM) # 显式声明支持 C 和 ASM

# 设置 C 标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# 定义 Build Type (默认为 Debug)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
message("Build type: " ${CMAKE_BUILD_TYPE})

# =========================================================
# 2. 编译器核心选项 (必须在 add_subdirectory 之前)
# =========================================================
# 浮点运算单元 (FPU) 设置
set(COMMON_FLAGS "-mfloat-abi=hard -mfpu=fpv5-d16")

# 内核架构设置 (关键：STM32H7 必须强制使用 Thumb 指令集)
set(MCU_FLAGS "-mthumb -mthumb-interwork -march=armv7e-m -mtune=cortex-m7")

# 将标志应用到所有语言
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS} ${MCU_FLAGS} -Wall -fdata-sections -ffunction-sections")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS} ${MCU_FLAGS} -Wall -fdata-sections -ffunction-sections")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${COMMON_FLAGS} ${MCU_FLAGS}")

# =========================================================
# 3. 源文件与目录配置
# =========================================================
add_executable(${CMAKE_PROJECT_NAME}
        USER/Components/Device/Src/dm_motor.c)

# 添加 CubeMX 生成的源码 (此时它会继承上面的编译器参数)
add_subdirectory(cmake/stm32cubemx)

# 自动搜索用户源文件
file(GLOB_RECURSE SOURCES
        "USER/BSP/Src/*.c"
        "USER/Components/Device/Src/*.c"
        "USER/Components/Algorithm/Src/*.c"
        "USER/Components/Controller/Src/*.c"
        "USER/Components/Message/*.c"
        "USER/Application/Task/Src/*.c"
        "USER/Application/Task/shoot/*.c"
        "USER/Application/Task/gimbal/*.c"
)

# 添加源文件到工程
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${SOURCES})

# 添加头文件路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        USER/Components/Device/Inc
        USER/Components/Algorithm/Inc
        USER/Components/Controller/Inc
        USER/Components/Message
        USER/Application/Task/Inc
        USER/Application/Task/shoot
        USER/Application/Task/gimbal
        USER/BSP/Inc
        Middlewares/ST/STM32_USB_Device_Library/Core/Inc
)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE DSP)

# =========================================================
# 4. 链接库与链接脚本 (修复报错的核心部分)
# =========================================================

# 移除隐式依赖冲突
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# 链接必要的库
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        arm_cortexM7lfdp_math
        m  # <--- 必须加 'm' (数学库)，解决 atan2f/acosf 报错
)

# --- 链接脚本路径设置 ---
# 获取当前目录下的 .ld 文件路径
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32H723XG_FLASH.ld")

# 检查脚本是否存在 (双重保险)
if(NOT EXISTS ${LINKER_SCRIPT})
    message(FATAL_ERROR "Could not find linker script at: ${LINKER_SCRIPT}")
endif()

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        # 核心修复：指定链接脚本
        -T ${LINKER_SCRIPT}

        # 生成 Map 文件
        -Wl,-Map=${CMAKE_PROJECT_NAME}.map

        # 垃圾回收（移除未使用的函数）
        -Wl,--gc-sections

        # 打印浮点数支持 (可选)
        -u _printf_float
)
# =========================================================
# 5. 后处理 (生成 Hex 文件)
# =========================================================
add_custom_command(
        TARGET ${CMAKE_PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
        COMMENT "Generating HEX file for STM32..."
)