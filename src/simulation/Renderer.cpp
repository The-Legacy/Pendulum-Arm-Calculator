#include "../../inlcude/pendulum/simulation/Renderer.hpp"
#include "raylib.h"
#include "../../inlcude/pendulum/utils/Config.hpp"
#include <cmath>
#include <string>

namespace pendulum {

Renderer::Renderer(int w, int h) : window_width(w), window_height(h) {}

Renderer::~Renderer() {
    shutdown();
}

bool Renderer::init() {
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(window_width, window_height, "Pendulum Arm Simulator");
    SetTargetFPS(60);  // 60 FPS rendering (independent of sim speed)
    initialized = true;
    return true;
}

bool Renderer::shouldClose() const {
    return WindowShouldClose();
}

void Renderer::shutdown() {
    if (initialized) {
        CloseWindow();
    }
}

void Renderer::render(const SystemState& state, float target_x, float target_y, float target_z,
                      float sim_time, int step_count, float speed_multiplier,
                      bool input_mode, const std::string& input_text) {
    // Handle camera orbit with mouse
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Vector2 delta = GetMouseDelta();
        cam_angle_h -= delta.x * 0.005f;
        cam_angle_v -= delta.y * 0.005f;
        // Clamp vertical angle to avoid flipping
        if (cam_angle_v < 0.1f) cam_angle_v = 0.1f;
        if (cam_angle_v > 1.5f) cam_angle_v = 1.5f;
    }

    // Zoom with scroll wheel
    float wheel = GetMouseWheelMove();
    cam_distance -= wheel * 0.1f;
    if (cam_distance < 0.3f) cam_distance = 0.3f;
    if (cam_distance > 5.0f) cam_distance = 5.0f;

    // Compute camera position from orbit angles
    Camera3D camera = {};
    camera.position = {
        cam_distance * std::cos(cam_angle_v) * std::cos(cam_angle_h),
        cam_distance * std::sin(cam_angle_v),
        cam_distance * std::cos(cam_angle_v) * std::sin(cam_angle_h)
    };
    camera.target = {0.0f, 0.2f, 0.0f};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    BeginDrawing();
    ClearBackground(RAYWHITE);

    BeginMode3D(camera);

    // Draw scene
    DrawGrid(20, 0.1f);                   // Grid
    drawCoordinateFrame();                // XYZ axes
    drawArm(state);                       // Arm segments
    drawClaw(state);                      // Claw endpoint
    drawTarget(target_x, target_y, target_z);  // Target position

    EndMode3D();

    // Draw HUD (2D text overlay)
    drawHUD(sim_time, step_count, speed_multiplier);
    drawControlsHelp();

    // Draw input overlay if in input mode
    if (input_mode) {
        drawInputOverlay(input_text);
    }

    EndDrawing();
}

void Renderer::drawArm(const SystemState& state) {
    // Extract joint angles
    float theta_turntable = state.theta[0];
    float theta_shoulder = state.theta[1];
    float theta_elbow = state.theta[2];
    float theta_wrist = state.theta[3];

    // Cumulative angles
    float pitch_s = theta_shoulder;
    float pitch_e = theta_shoulder + theta_elbow;
    float pitch_w = theta_shoulder + theta_elbow + theta_wrist;

    // Tangential direction (perpendicular to radial plane, around Z axis)
    float tang_x = -std::sin(theta_turntable);
    float tang_z =  std::cos(theta_turntable);

    // Shoulder joint at origin
    Vector3 shoulder_pos = {0.0f, 0.0f, 0.0f};

    // End of shoulder link (in radial plane)
    float r1 = SHOULDER_TO_ELBOW * std::cos(pitch_s);
    float z1 = SHOULDER_TO_ELBOW * std::sin(pitch_s);
    Vector3 elbow_tip = {
        r1 * std::cos(theta_turntable), z1, r1 * std::sin(theta_turntable)
    };

    // Elbow offset: tangential jog (out the side of the arm)
    Vector3 elbow_pos = {
        elbow_tip.x + ELBOW_JOINT_OFFSET * tang_x,
        elbow_tip.y,
        elbow_tip.z + ELBOW_JOINT_OFFSET * tang_z
    };

    // End of forearm (in radial plane, starting from r1/z1 since offsets don't affect r,z)
    float r2 = r1 + ELBOW_TO_WRIST * std::cos(pitch_e);
    float z2 = z1 + ELBOW_TO_WRIST * std::sin(pitch_e);
    Vector3 wrist_tip = {
        r2 * std::cos(theta_turntable) + ELBOW_JOINT_OFFSET * tang_x,
        z2,
        r2 * std::sin(theta_turntable) + ELBOW_JOINT_OFFSET * tang_z
    };

    // Wrist offset: tangential jog (stacks with elbow offset)
    float total_offset = ELBOW_JOINT_OFFSET + WRIST_JOINT_OFFSET;
    Vector3 wrist_pos = {
        r2 * std::cos(theta_turntable) + total_offset * tang_x,
        z2,
        r2 * std::sin(theta_turntable) + total_offset * tang_z
    };

    // Claw endpoint
    float r_claw = r2 + WRIST_TO_CLAW * std::cos(pitch_w);
    float z_claw = z2 + WRIST_TO_CLAW * std::sin(pitch_w);
    Vector3 claw_pos = {
        r_claw * std::cos(theta_turntable) + total_offset * tang_x,
        z_claw,
        r_claw * std::sin(theta_turntable) + total_offset * tang_z
    };

    // Draw link segments: shoulder link, tangential offset jog, forearm, tangential offset jog, wrist link
    DrawCapsule(shoulder_pos, elbow_tip, 0.03f, 8, 8, ColorFromHSV(120.0f, 0.7f, 0.9f));  // shoulder
    DrawCapsule(elbow_tip, elbow_pos, 0.02f, 8, 8, ColorFromHSV(100.0f, 0.7f, 0.9f));     // elbow offset
    DrawCapsule(elbow_pos, wrist_tip, 0.025f, 8, 8, ColorFromHSV(180.0f, 0.7f, 0.9f));    // forearm
    DrawCapsule(wrist_tip, wrist_pos, 0.015f, 8, 8, ColorFromHSV(160.0f, 0.7f, 0.9f));    // wrist offset
    DrawCapsule(wrist_pos, claw_pos, 0.02f, 8, 8, ColorFromHSV(240.0f, 0.7f, 0.9f));      // wrist link

    // Draw joints as spheres
    DrawSphere(shoulder_pos, 0.04f, ColorFromHSV(0.0f, 0.8f, 1.0f));
    DrawSphere(elbow_pos, 0.035f, ColorFromHSV(60.0f, 0.8f, 1.0f));
    DrawSphere(wrist_pos, 0.03f, ColorFromHSV(120.0f, 0.8f, 1.0f));
}

void Renderer::drawLink(float r_start, float z_start, float r_end, float z_end,
                        float radius, float turntable_angle, float hue) {
    Vector3 p1 = {
        r_start * std::cos(turntable_angle),
        z_start,
        r_start * std::sin(turntable_angle)
    };
    Vector3 p2 = {
        r_end * std::cos(turntable_angle),
        z_end,
        r_end * std::sin(turntable_angle)
    };

    DrawCapsule(p1, p2, radius, 8, 8, ColorFromHSV(hue, 0.7f, 0.9f));
}

void Renderer::drawSphere(float x, float y, float z, float radius, float hue) {
    DrawSphere({x, y, z}, radius, ColorFromHSV(hue, 0.8f, 1.0f));
}

void Renderer::drawClaw(const SystemState& state) {
    float theta_turntable = state.theta[0];
    float theta_shoulder = state.theta[1];
    float theta_elbow = state.theta[2];
    float theta_wrist = state.theta[3];

    float pitch_s = theta_shoulder;
    float pitch_e = theta_shoulder + theta_elbow;
    float pitch_w = theta_shoulder + theta_elbow + theta_wrist;

    // Tangential direction
    float tang_x = -std::sin(theta_turntable);
    float tang_z =  std::cos(theta_turntable);
    float total_offset = ELBOW_JOINT_OFFSET + WRIST_JOINT_OFFSET;

    // Planar kinematics (no offsets in r,z)
    float r1 = SHOULDER_TO_ELBOW * std::cos(pitch_s);
    float z1 = SHOULDER_TO_ELBOW * std::sin(pitch_s);
    float r2 = r1 + ELBOW_TO_WRIST * std::cos(pitch_e);
    float z2 = z1 + ELBOW_TO_WRIST * std::sin(pitch_e);
    float r_claw = r2 + WRIST_TO_CLAW * std::cos(pitch_w);
    float z_claw = z2 + WRIST_TO_CLAW * std::sin(pitch_w);

    Vector3 claw = {
        r_claw * std::cos(theta_turntable) + total_offset * tang_x,
        z_claw,
        r_claw * std::sin(theta_turntable) + total_offset * tang_z
    };

    DrawCube(claw, 0.05f, 0.05f, 0.08f, RED);
}

void Renderer::drawTarget(float target_x, float target_y, float target_z) {
    DrawSphere({target_x, target_z, target_y}, 0.02f, GREEN);
    DrawCube({target_x, target_z, target_y}, 0.03f, 0.03f, 0.03f, LIME);
}

void Renderer::drawCoordinateFrame() {
    DrawLine3D({0, 0, 0}, {0.3f, 0, 0}, RED);    // X
    DrawLine3D({0, 0, 0}, {0, 0.3f, 0}, GREEN);  // Y
    DrawLine3D({0, 0, 0}, {0, 0, 0.3f}, BLUE);   // Z
}

void Renderer::drawHUD(float sim_time, int step_count, float speed_multiplier) {
    DrawFPS(10, 10);
    DrawText(TextFormat("Sim Time: %.2f s", sim_time), 10, 40, 20, BLACK);
    DrawText(TextFormat("Steps: %d", step_count), 10, 70, 20, BLACK);
    DrawText(TextFormat("Speed: %.1fx", speed_multiplier), 10, 100, 20, DARKBLUE);
}

void Renderer::drawInputOverlay(const std::string& input_text) {
    int box_w = 400, box_h = 60;
    int box_x = (window_width - box_w) / 2;
    int box_y = window_height - 100;
    DrawRectangle(box_x - 5, box_y - 30, box_w + 10, box_h + 35, Fade(BLACK, 0.7f));
    DrawText("Enter target: x y z [wrist_pitch]", box_x, box_y - 25, 16, WHITE);
    DrawRectangle(box_x, box_y, box_w, box_h, DARKGRAY);
    DrawRectangleLines(box_x, box_y, box_w, box_h, SKYBLUE);
    std::string display = input_text + "_";
    DrawText(display.c_str(), box_x + 10, box_y + 18, 24, WHITE);
}

void Renderer::drawControlsHelp() {
    int y = window_height - 160;
    int x = 10;
    DrawText("[T] New Target  [+/-] Speed  [R] Reset  [RMB] Orbit  [Scroll] Zoom", x, y, 16, DARKGRAY);
}

} 