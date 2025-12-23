
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <string>
#include <vector>
#include <cmath>

class Graphics
{
public:
    Graphics(int width, int height, const std::string &robot_image_path, const std::string &map_image_path);
    ~Graphics();

    void drawRobot(SDL_Surface *screen, double robot_x, double robot_y, double heading);
    void drawSensorData(SDL_Surface *screen, const std::vector<std::pair<int, int>> &point_cloud, 
                        double start_x, double start_y);
    void drawMap(SDL_Surface *screen);

    int getWidth() const { return width; }
    int getHeight() const { return height; }

private:
    int width, height;
    SDL_Surface *robot_image;
    SDL_Surface *map_image;
    SDL_Rect map_rect;
    
    SDL_Surface *rotateSurface(SDL_Surface *src, double angle);
    void drawLine(SDL_Surface *screen, int x1, int y1, int x2, int y2, uint32_t color);
    void drawCircle(SDL_Surface *screen, int cx, int cy, int radius, uint32_t color);
    uint32_t getPixel(int x, int y) const;
    void putPixel(SDL_Surface *surface, int x, int y, uint32_t color);
};
