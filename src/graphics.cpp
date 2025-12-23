
#include <cmath>
#include <iostream>
#include <algorithm>

#include "robot_simulator_pkg/graphics.hpp"

constexpr double ROBOT_SIZE_RATIO = 0.1;

Graphics::Graphics(int w, int h, const std::string &robot_image_path, const std::string &map_image_path)
    : width(w), height(h), robot_image(nullptr), map_image(nullptr)
{
    map_rect.x = 0;
    map_rect.y = 0;
    map_rect.w = 0;
    map_rect.h = 0;

    SDL_Surface *temp_robot = IMG_Load(robot_image_path.c_str());
    if (temp_robot)
    {
        int new_w = std::max(1, static_cast<int>(temp_robot->w * ROBOT_SIZE_RATIO));
        int new_h = std::max(1, static_cast<int>(temp_robot->h * ROBOT_SIZE_RATIO));

        robot_image = SDL_CreateRGBSurfaceWithFormat(0, new_w, new_h, 32, SDL_PIXELFORMAT_RGBA8888);

        SDL_BlitScaled(temp_robot, nullptr, robot_image, nullptr);

        SDL_FreeSurface(temp_robot);
    }
    else
    {
        robot_image = SDL_CreateRGBSurfaceWithFormat(0, 40, 40, 32, SDL_PIXELFORMAT_RGBA8888);
        SDL_FillRect(robot_image, nullptr, SDL_MapRGB(robot_image->format, 255, 0, 0));
    }

    SDL_Surface *temp_map = IMG_Load(map_image_path.c_str());
    if (temp_map)
    {
        double scale_ratio = std::min(static_cast<double>(width) / temp_map->w,
                                      static_cast<double>(height) / temp_map->h);
        int new_w = std::max(1, static_cast<int>(temp_map->w * scale_ratio));
        int new_h = std::max(1, static_cast<int>(temp_map->h * scale_ratio));

        map_image = SDL_CreateRGBSurfaceWithFormat(0, new_w, new_h, 32, SDL_PIXELFORMAT_RGBA8888);

        SDL_BlitScaled(temp_map, nullptr, map_image, nullptr);

        map_rect.x = (width - new_w) / 2;
        map_rect.y = (height - new_h) / 2;
        map_rect.w = new_w;
        map_rect.h = new_h;

        SDL_FreeSurface(temp_map);
    }
    else
    {
        map_image = SDL_CreateRGBSurfaceWithFormat(0, width, height, 32, SDL_PIXELFORMAT_RGBA8888);
        SDL_FillRect(map_image, nullptr, SDL_MapRGB(map_image->format, 255, 255, 255));
        map_rect.w = width;
        map_rect.h = height;
    }
}

Graphics::~Graphics()
{
    if (robot_image)
        SDL_FreeSurface(robot_image);
    if (map_image)
        SDL_FreeSurface(map_image);
}

void Graphics::drawMap(SDL_Surface *screen)
{
    if (map_image)
        SDL_BlitSurface(map_image, nullptr, screen, &map_rect);
}

void Graphics::drawRobot(SDL_Surface *screen, double robot_x, double robot_y, double heading)
{
    if (!robot_image)
        return;

    // Rotate the robot image
    // Convert radians to degrees (SDL uses degrees)
    // Note: SDL2_gfx usually handles rotation, but here we do manual or use a helper.
    // Since we don't have SDL2_gfx linked in the CMakeLists efficiently,
    // let's use the safer manual rotation provided below.

    SDL_Surface *rotated = rotateSurface(robot_image, -heading * 180.0 / M_PI);

    if (rotated)
    {
        SDL_Rect dest;
        dest.x = static_cast<int>(robot_x - rotated->w / 2);
        dest.y = static_cast<int>(robot_y - rotated->h / 2);
        SDL_BlitSurface(rotated, nullptr, screen, &dest);
        SDL_FreeSurface(rotated);
    }
}

SDL_Surface *Graphics::rotateSurface(SDL_Surface *src, double angle)
{
    if (!src)
        return nullptr;

    double rad = angle * M_PI / 180.0;
    double c = std::cos(rad);
    double s = std::sin(rad);

    int new_w = static_cast<int>(std::abs(src->w * c) + std::abs(src->h * s));
    int new_h = static_cast<int>(std::abs(src->w * s) + std::abs(src->h * c));

    SDL_Surface *dst = SDL_CreateRGBSurfaceWithFormat(0, new_w, new_h, 32, SDL_PIXELFORMAT_RGBA8888);
    if (!dst)
        return nullptr;

    if (SDL_MUSTLOCK(src))
        SDL_LockSurface(src);
    if (SDL_MUSTLOCK(dst))
        SDL_LockSurface(dst);

    uint32_t *src_pixels = (uint32_t *)src->pixels;
    uint32_t *dst_pixels = (uint32_t *)dst->pixels;

    int src_pitch_pixels = src->pitch / 4;
    int dst_pitch_pixels = dst->pitch / 4;

    double cx = src->w / 2.0;
    double cy = src->h / 2.0;
    double dest_cx = new_w / 2.0;
    double dest_cy = new_h / 2.0;

    for (int y = 0; y < new_h; y++)
    {
        for (int x = 0; x < new_w; x++)
        {
            double target_x = (x - dest_cx) * c - (y - dest_cy) * s + cx;
            double target_y = (x - dest_cx) * s + (y - dest_cy) * c + cy;

            int src_x = static_cast<int>(target_x);
            int src_y = static_cast<int>(target_y);

            if (src_x >= 0 && src_x < src->w && src_y >= 0 && src_y < src->h)
            {
                uint32_t pixel = src_pixels[src_y * src_pitch_pixels + src_x];
                dst_pixels[y * dst_pitch_pixels + x] = pixel;
            }
            else
            {
                dst_pixels[y * dst_pitch_pixels + x] = 0x00000000;
            }
        }
    }

    if (SDL_MUSTLOCK(src))
        SDL_UnlockSurface(src);
    if (SDL_MUSTLOCK(dst))
        SDL_UnlockSurface(dst);

    return dst;
}

void Graphics::drawSensorData(SDL_Surface *screen, const std::vector<std::pair<int, int>> &point_cloud,
                              double start_x, double start_y)
{
    uint32_t cyan = SDL_MapRGB(screen->format, 0, 200, 255);
    uint32_t red = SDL_MapRGB(screen->format, 255, 0, 0);

    for (const auto &p : point_cloud)
    {
        drawLine(screen, (int)start_x, (int)start_y, p.first, p.second, cyan);
        drawCircle(screen, p.first, p.second, 3, red);
    }
}

void Graphics::putPixel(SDL_Surface *surface, int x, int y, uint32_t color)
{
    if (x < 0 || x >= surface->w || y < 0 || y >= surface->h)
        return;

    int bpp = surface->format->BytesPerPixel;
    uint8_t *p = (uint8_t *)surface->pixels + y * surface->pitch + x * bpp;

    switch (bpp)
    {
    case 1:
        *p = color;
        break;
    case 2:
        *(uint16_t *)p = color;
        break;
    case 3:
        if (SDL_BYTEORDER == SDL_BIG_ENDIAN)
        {
            p[0] = (color >> 16) & 0xff;
            p[1] = (color >> 8) & 0xff;
            p[2] = color & 0xff;
        }
        else
        {
            p[0] = color & 0xff;
            p[1] = (color >> 8) & 0xff;
            p[2] = (color >> 16) & 0xff;
        }
        break;
    case 4:
        *(uint32_t *)p = color;
        break;
    }
}

void Graphics::drawLine(SDL_Surface *screen, int x1, int y1, int x2, int y2, uint32_t color)
{
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
        putPixel(screen, x1, y1, color);
        if (x1 == x2 && y1 == y2)
            break;
        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y1 += sy;
        }
    }
}

void Graphics::drawCircle(SDL_Surface *screen, int cx, int cy, int radius, uint32_t color)
{
    int x = radius;
    int y = 0;
    int err = 0;

    while (x >= y)
    {
        putPixel(screen, cx + x, cy + y, color);
        putPixel(screen, cx + y, cy + x, color);
        putPixel(screen, cx - y, cy + x, color);
        putPixel(screen, cx - x, cy + y, color);
        putPixel(screen, cx - x, cy - y, color);
        putPixel(screen, cx - y, cy - x, color);
        putPixel(screen, cx + y, cy - x, color);
        putPixel(screen, cx + x, cy - y, color);

        if (err <= 0)
        {
            y += 1;
            err += 2 * y + 1;
        }
        else
        {
            x -= 1;
            err -= 2 * x + 1;
        }
    }
}
uint32_t Graphics::getPixel(int x, int y) const { return 0; }