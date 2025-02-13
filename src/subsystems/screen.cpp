#include "robot/screen.h"
#include "pros/apix.h"
#include "liblvgl/lvgl.h"
#include "globals.h"

using namespace Robot;
using namespace Robot::Globals;

// Screen and UI objects
lv_obj_t *auton_select_screen, *info_label;
lv_obj_t *field_btns[10];

lv_obj_t *x_label, *y_label, *theta_label;

void update_position_labels(float x, float y, float theta) {
    lv_label_set_text_fmt(x_label, "X: %.2f", x);
    lv_label_set_text_fmt(y_label, "Y: %.2f", y);
    lv_label_set_text_fmt(theta_label, "Theta: %.2f", theta);
}

// Helper function to update field button styles based on selection
void update_fields() {
    static lv_style_t selected_style, not_selected_style;
    lv_style_init(&selected_style); 
    lv_style_init(&not_selected_style);
    lv_style_set_bg_opa(&selected_style, LV_OPA_100);
    lv_style_set_bg_opa(&not_selected_style, LV_OPA_40);

    for (int i = 0; i < 10; i++) {
        lv_obj_t *btn = field_btns[i];
        if (i == active_route) {
            lv_obj_add_style(btn, &selected_style, 0);
        } else {
            lv_obj_add_style(btn, &not_selected_style, 0);
        }
    }
}

// Update info label with the clicked field's name
void set_info_label(const char* s) {
    lv_label_set_text_fmt(info_label, "Clicked %s", s);
}

// event callbacks 
void red_neg_cb(lv_event_t *e) { set_info_label("red neg"); active_route = RED_NEG; playingRedSide = true; update_fields(); }
void red_pos_cb(lv_event_t *e) { set_info_label("red pos"); active_route = RED_POS; playingRedSide = true; update_fields(); }
void blue_neg_cb(lv_event_t *e) { set_info_label("blue neg"); active_route = BLUE_NEG; playingRedSide = false; update_fields(); }
void blue_pos_cb(lv_event_t *e) { set_info_label("blue pos"); active_route = BLUE_POS; playingRedSide = false; update_fields(); }
void red_goal_cb(lv_event_t *e) { set_info_label("red goal"); active_route = RED_GOALRUSH; playingRedSide = true; update_fields(); }
void blue_goal_cb(lv_event_t *e) { set_info_label("blue goal"); active_route = BLUE_GOALRUSH; playingRedSide = false; update_fields(); }
void red_ring_cb(lv_event_t *e) { set_info_label("red ring"); active_route = RED_RING; playingRedSide = true; update_fields(); }
void blue_ring_cb(lv_event_t *e) { set_info_label("blue ring"); active_route = BLUE_RING; playingRedSide = false; update_fields();}
void skills_cb(lv_event_t *e) { set_info_label("skills"); active_route = SKILLS; update_fields(); }
void none_cb(lv_event_t *e) { set_info_label("none"); active_route = NONE; update_fields(); }


// helper function to create buttons
lv_obj_t* make_button(lv_obj_t* holder, int posX, int posY, int width, int height, const char* s, std::string color, void (*callback)(lv_event_t*)) {
    lv_obj_t* btn = lv_btn_create(holder);
    lv_obj_set_pos(btn, posX, posY);
    lv_obj_set_size(btn, width, height);
    lv_obj_add_event_cb(btn, callback, LV_EVENT_CLICKED, NULL);

    lv_obj_t* label = lv_label_create(btn);
    lv_label_set_text(label, s);
    lv_obj_center(label);

    static lv_style_t red_style, blue_style, gray_style;
    lv_style_init(&red_style);
    lv_style_init(&blue_style);
    lv_style_init(&gray_style);

    lv_style_set_bg_color(&red_style, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_bg_color(&blue_style, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_color(&gray_style, lv_palette_main(LV_PALETTE_GREY));

    if (color == "red") {
        lv_obj_add_style(btn, &red_style, 0);
    }
    else if (color == "blue") {
        lv_obj_add_style(btn, &blue_style, 0);
    }
    else {
        lv_obj_add_style(btn, &gray_style, 0);
    }

    return btn;
}

void runScreen() {
    lv_init();

    // Create the autonomous selection screen
    auton_select_screen = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(auton_select_screen);
    lv_obj_set_size(auton_select_screen, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
    lv_obj_center(auton_select_screen);

    // Info label 
    info_label = lv_label_create(auton_select_screen);
    lv_label_set_text(info_label, "Choose field");
    lv_obj_align(info_label, LV_ALIGN_TOP_MID, 0, 10);

    // Create buttons 
    field_btns[0] = make_button(auton_select_screen, 50, 50, 50, 50, "Neg", "red", red_neg_cb);
    field_btns[1] = make_button(auton_select_screen, 120, 50, 50, 50, "Pos", "red", red_pos_cb);
    field_btns[2] = make_button(auton_select_screen, 190, 50, 50, 50, "Goal", "red", red_goal_cb);
    field_btns[3] = make_button(auton_select_screen, 260, 50, 50, 50, "Ring", "red", red_ring_cb);
    field_btns[4] = make_button(auton_select_screen, 50, 120, 50, 50, "Neg", "blue", blue_neg_cb);
    field_btns[5] = make_button(auton_select_screen, 120, 120, 50, 50, "Pos", "blue", blue_pos_cb);
    field_btns[6] = make_button(auton_select_screen, 190, 120, 50, 50, "Goal", "blue", blue_goal_cb);
    field_btns[7] = make_button(auton_select_screen, 260, 120, 50, 50, "Ring", "blue", blue_ring_cb);
    field_btns[8] = make_button(auton_select_screen, 330, 50, 50, 50, "Skills", "gray", skills_cb);
    field_btns[9] = make_button(auton_select_screen, 330, 120, 50, 50, "None", "gray", none_cb);


    // x_label = lv_label_create(auton_select_screen);
    // lv_label_set_text(x_label, "X: 0.00");
    // lv_obj_align(x_label, LV_ALIGN_TOP_RIGHT, -10, 10);

    // y_label = lv_label_create(auton_select_screen);
    // lv_label_set_text(y_label, "Y: 0.00");
    // lv_obj_align(y_label, LV_ALIGN_TOP_RIGHT, -10, 40);

    // theta_label = lv_label_create(auton_select_screen);
    // lv_label_set_text(theta_label, "Theta: 0.00");
    // lv_obj_align(theta_label, LV_ALIGN_TOP_RIGHT, -10, 70);
}
