#include <stdio.h>
#include <uptime.h>
#include "lua/lua.h"
#include "lua/lauxlib.h"
#include "lua/lualib.h"

#include "position_integration.h"
#include "control.h"
#include "match.h"


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
    lua_Number x, y, watch_x, watch_y;
    match_set_disable_position_control(false);

    if (lua_gettop(l) < 2) return 0;
    if (lua_gettop(l) < 3) {
        x = lua_tonumber(l, -2);
        y = lua_tonumber(l, -1);
        goto_position(x, y, 0, 0);
        control_update_setpoint_vx(0);
        control_update_setpoint_vy(0);
        control_update_setpoint_omega(0);
        return 0;
    }
    if (lua_gettop(l) < 4) return 0;

    x = lua_tonumber(l, -4);
    y = lua_tonumber(l, -3);
    watch_x = lua_tonumber(l, -2);
    watch_y = lua_tonumber(l, -1);
    goto_position(x, y, watch_x, watch_y);
    control_update_setpoint_vx(0);
    control_update_setpoint_vy(0);
    control_update_setpoint_omega(0);

    return 0;
}

int cmd_m(lua_State *l)
{
    lua_Number x, y, watch_x, watch_y;
    match_set_disable_position_control(false);

    if (lua_gettop(l) < 2) return 0;

    x = lua_tonumber(l, -2);
    y = lua_tonumber(l, -1);
    goto_position(x, y, 100, 0);
    control_update_setpoint_vx(0);
    control_update_setpoint_vy(0);
    control_update_setpoint_omega(0);

    return 0;
}

int cmd_position_reset_to(lua_State *l)
{
    lua_Number x, y, theta;
    if (lua_gettop(l) < 3) return 0;

    x = lua_tonumber(l, -3);
    y = lua_tonumber(l, -2);
    theta = lua_tonumber(l, -1);

    position_reset_to(x, y, theta);

    return 0;
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
    ready_for_match();
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

    lua_pushcfunction(l, cmd_m);
    lua_setglobal(l, "m");
}

