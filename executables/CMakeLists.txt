add_executable(Model
        main.cpp
        )

add_executable(drawPendulum
        drawPendulum.cpp)

target_link_libraries(
        Model
        ${OpenCV_LIBS}
)

target_link_libraries(
        drawPendulum
        ${OpenCV_LIBS}
)
