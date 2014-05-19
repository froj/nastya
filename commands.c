#include <stdio.h>
#include <uptime.h>
#include "lua/lua.h"
#include "lua/lauxlib.h"
#include "lua/lualib.h"

#include "position_integration.h"
#include "control.h"
#include "match.h"
#include "drive.h"
#include "param.h"


int cmd_get_pos(lua_State *l)
{
    float x, y;
    get_position(&x, &y);
    printf("lua command %f %f\n", x, y);
    lua_pushnumber(l, (lua_Number)x);
    lua_pushnumber(l, (lua_Number)y);
    return 2;
}

int cmd_get_pos_x(lua_State *l)
{
    float x, y;
    get_position(&x, &y);
    lua_pushnumber(l, (lua_Number)x);
    return 1;
}

int cmd_get_pos_y(lua_State *l)
{
    float x, y;
    get_position(&x, &y);
    lua_pushnumber(l, (lua_Number)y);
    return 1;
}

int cmd_get_pos_theta(lua_State *l)
{
    float theta;
    theta = get_heading();
    lua_pushnumber(l, (lua_Number)theta);
    return 1;
}

int cmd_goto_position(lua_State *l)
{
    lua_Number x, y;
    if (lua_gettop(l) == 2) {
        x = lua_tonumber(l, -2);
        y = lua_tonumber(l, -1);
        drive_goto(x, y);
        // control_update_setpoint_vx(0);
        // control_update_setpoint_vy(0);
        // control_update_setpoint_omega(0);
        return 0;
    }
    return 0;
}

int cmd_lookat(lua_State *l)
{
    lua_Number x, y;
    if (lua_gettop(l) == 2) {
        x = lua_tonumber(l, -2);
        y = lua_tonumber(l, -1);
        drive_set_look_at(x, y);
        return 0;
    }
}

int cmd_heading(lua_State *l)
{
    lua_Number a;
    if (lua_gettop(l) == 1) {
        a = lua_tonumber(l, -1);
        drive_set_heading(a);
        return 0;
    }
}

int cmd_position_reset_to(lua_State *l)
{
    lua_Number x, y, theta;
    if (lua_gettop(l) < 3) {
        lua_pushstring(l, "expected: x,y,theta");
        return 1;
    }

    x = lua_tonumber(l, -3);
    y = lua_tonumber(l, -2);
    theta = lua_tonumber(l, -1);

    position_reset_to(x, y, theta);
    drive_set_dest(x,y);
    lua_pushstring(l, "OK");
    return 1;
}

int cmd_set_red(lua_State *l)
{
    match_set_red();
    return 0;
}

int cmd_set_yellow(lua_State *l)
{
    match_set_yellow();
    return 0;
}

int cmd_match(lua_State *l)
{
    match_restart(false);
    return 0;
}

int cmd_control_on(lua_State *l)
{
    control_update_setpoint_vx(0);
    control_update_setpoint_vy(0);
    control_update_setpoint_omega(0);
    nastya_cs.vx_control_enable = true;
    nastya_cs.vy_control_enable = true;
    nastya_cs.omega_control_enable = true;
    return 0;
}

int cmd_control_off(lua_State *l)
{

    nastya_cs.vx_control_enable = false;
    nastya_cs.vy_control_enable = false;
    nastya_cs.omega_control_enable = false;
    return 0;
}

int cmd_set_param(lua_State *l)
{
    if (lua_gettop(l) == 2) {
        const char *name = lua_tostring(l, -2);
        lua_Number val = lua_tonumber(l, -1);
        if (param_set_by_name(name, val))
            lua_pushstring(l, "OK");
        else
            lua_pushstring(l, "Param not found.");
    }
    return 1;
}

int cmd_get_param(lua_State *l)
{
    if (lua_gettop(l) == 1) {
        const char *name = lua_tostring(l, -1);
        double val;
        if (param_read_by_name(name, &val))
            lua_pushnumber(l, val);
        else
            lua_pushstring(l, "Param not found.");
    }
    return 1;
}

int cmd_list_param(lua_State *l)
{
    static char buf[10000];
    if (param_list(buf, sizeof(buf)) == 0) {
        lua_pushstring(l, buf);
        return 1;
    } else {
        lua_pushstring(l, "buffer too small");
        return 1;
    }
}

void commands_register(lua_State *l)
{
    lua_pushcfunction(l, cmd_get_pos);
    lua_setglobal(l, "get_pos");

    lua_pushcfunction(l, cmd_get_pos_x);
    lua_setglobal(l, "x");

    lua_pushcfunction(l, cmd_get_pos_y);
    lua_setglobal(l, "y");

    lua_pushcfunction(l, cmd_get_pos_theta);
    lua_setglobal(l, "r");

    lua_pushcfunction(l, cmd_goto_position);
    lua_setglobal(l, "move");

    lua_pushcfunction(l, cmd_position_reset_to);
    lua_setglobal(l, "reset_to");

    lua_pushcfunction(l, cmd_set_red);
    lua_setglobal(l, "red");

    lua_pushcfunction(l, cmd_set_yellow);
    lua_setglobal(l, "yellow");

    lua_pushcfunction(l, cmd_match);
    lua_setglobal(l, "match");

    lua_pushcfunction(l, cmd_control_on);
    lua_setglobal(l, "con");

    lua_pushcfunction(l, cmd_control_off);
    lua_setglobal(l, "coff");

    lua_pushcfunction(l, cmd_set_param);
    lua_setglobal(l, "param_set");

    lua_pushcfunction(l, cmd_get_param);
    lua_setglobal(l, "param_get");

    lua_pushcfunction(l, cmd_list_param);
    lua_setglobal(l, "param_list");

    lua_pushcfunction(l, cmd_lookat);
    lua_setglobal(l, "lookat");

    lua_pushcfunction(l, cmd_heading);
    lua_setglobal(l, "heading");
}

