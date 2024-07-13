SET(SOURCES ui_Screen1.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    ui_img_gradiente_png.c
    ui_img_clockwise_sec_png.c
    ui_img_warningsignal2_png.c
    ui_img_arariboialogo2_png.c)

add_library(ui ${SOURCES})
