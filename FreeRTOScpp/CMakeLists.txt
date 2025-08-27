add_library(FreeRTOScpp OBJECT
    ./CallBack.cpp
    ./Lock.cpp
    ./ReadWrite.cpp
    ./TaskCpp.cpp
)

target_include_directories(FreeRTOScpp PUBLIC
    ./include
)

# Link with FreeRTOS kernel to access FreeRTOS.h
target_link_libraries(FreeRTOScpp PUBLIC
    FreeRTOS
)
