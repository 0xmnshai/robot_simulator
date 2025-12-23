#include <tuple>
#include <vector>
#include <SDL2/SDL.h>

class Ultrasonic
{
public:
    Ultrasonic(int range, double fov, SDL_Surface *surface);
    std::vector<std::tuple<int, int, int>> sense(double x, double y, double heading);

private:
    int range;
    double fov;
    SDL_Surface *surface;
};
