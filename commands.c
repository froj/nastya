#include <stdio.h>
#include <uptime.h>
#include "lua/lua.h"
#include "lua/lauxlib.h"
#include "lua/lualib.h"

#include "match.h"
#include "position_integration.h"


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

int cmd_get_pos_omega(lua_State *l)
{
    float omega;
    omega = get_omega();
    lua_pushnumber(l, (lua_Number)omega);
    return 1;
}

int cmd_goto_position(lua_State *l)
{
    lua_Number x, y, watch_x, watch_y;
    if (lua_gettop(l) < 2) return 0;
    if (lua_gettop(l) < 3) {
        x = lua_tonumber(l, -2);
        y = lua_tonumber(l, -1);
        goto_position(x, y, 0, 0);
        return 0;
    }
    if (lua_gettop(l) < 4) return 0;

    x = lua_tonumber(l, -4);
    y = lua_tonumber(l, -3);
    watch_x = lua_tonumber(l, -2);
    watch_y = lua_tonumber(l, -1);
    goto_position(x, y, watch_x, watch_y);

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

void commands_register(lua_State *l)
{
    lua_pushcfunction(l, cmd_get_pos);
    lua_setglobal(l, "get_pos");

    lua_pushcfunction(l, cmd_get_pos_x);
    lua_setglobal(l, "x");

    lua_pushcfunction(l, cmd_get_pos_y);
    lua_setglobal(l, "y");

    lua_pushcfunction(l, cmd_get_pos_omega);
    lua_setglobal(l, "o");

    lua_pushcfunction(l, cmd_position_reset_to);
    lua_setglobal("reset_to");
}

