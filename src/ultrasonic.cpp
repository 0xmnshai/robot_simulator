#include <cmath>
#include <iostream>

#include "robot_simulator_pkg/ultrasonic.hpp"

Ultrasonic::Ultrasonic(int r, double f, SDL_Surface *s)
    : range(r), fov(f), surface(s) {}

std::vector<std::tuple<int, int, int>>
Ultrasonic::sense(double x, double y, double heading)
{
    std::vector<std::tuple<int, int, int>> hits;

    if (!surface)
        return hits;
    if (SDL_MUSTLOCK(surface))
    {
        SDL_LockSurface(surface);
    }

    int width = surface->w;
    int height = surface->h;
    int bpp = surface->format->BytesPerPixel;
    int pitch = surface->pitch;
    void *pixels = surface->pixels;

    for (int i = 0; i < 60; i++)
    {
        double a = heading - fov + i * (2 * fov / 60.0);
        double cos_a = std::cos(a);
        double sin_a = std::sin(a);

        for (int r = 0; r < range; r += 4)
        {
            int tx = static_cast<int>(x + r * cos_a);
            int ty = static_cast<int>(y - r * sin_a);

            if (tx < 0 || ty < 0 || tx >= width || ty >= height)
                break;
 
            Uint32 pixel_color = 0;
            Uint8 *p = (Uint8 *)pixels + ty * pitch + tx * bpp;

            switch (bpp)
            {
            case 1:
                pixel_color = *p;
                break;
            case 2:
                pixel_color = *(Uint16 *)p;
                break;
            case 3:
                if (SDL_BYTEORDER == SDL_BIG_ENDIAN)
                    pixel_color = p[0] << 16 | p[1] << 8 | p[2];
                else
                    pixel_color = p[0] | p[1] << 8 | p[2] << 16;
                break;
            case 4:
                pixel_color = *(Uint32 *)p;
                break;
            }

            Uint8 r_val, g_val, b_val;
            SDL_GetRGB(pixel_color, surface->format, &r_val, &g_val, &b_val);
 
            if (r_val < 40 && g_val < 40 && b_val < 40)
            {
                hits.emplace_back(tx, ty, r);
                break;
            }
        }
    }

    if (SDL_MUSTLOCK(surface))
    {
        SDL_UnlockSurface(surface);
    }

    return hits;
}