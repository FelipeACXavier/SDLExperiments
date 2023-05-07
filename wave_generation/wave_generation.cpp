#include <iostream>
#include <vector>
#include <random>

#include "linalg.h"

#include <SDL2/SDL.h>

static const float FPS = 60.0;
#define WIDTH 1240
#define HEIGHT 600
#define WAVES true
#define RENDER_ALL false
#define DEFINED_WAVE true

int main()
{
  srand(time(NULL));

  std::vector<double> frequencies;
  std::vector<SDL_Color> colors;
  std::vector<std::vector<linalg::Double2d>> points;
  std::vector<linalg::Double2d> signals;

  bool run = true;

  int scale = 8;
  int waves = 50;
  int centerX = WIDTH / 12;
  int centerY = HEIGHT / 2;

  double phase = 0;
  double multi = 1;
  double mag = (4 / M_PI) * 20;

  // Initialize all randomized parameters
  for (int i = 1; i < waves + 1; i++)
  {
#if DEFINED_WAVE
    // For specific waves, now set to square wave
    signals.push_back(linalg::Double2d((mag / multi), phase, linalg::Format::Polar));
    frequencies.push_back(0.01 * multi * 2);
    multi += 2;
#else
    // For random waves
    signals.push_back(linalg::Double2d(i, phase, linalg::Format::Polar));
    frequencies.push_back(double(rand() % 200 - 100) / 1000);
    phase += M_PI / (rand() % 10 + 1);
#endif

    colors.push_back({uint8_t(rand() % 255),
                      uint8_t(rand() % 255),
                      uint8_t(rand() % 255),
                      (uint8_t)std::min((rand() % 50) * (i + 1), 255) });

    points.push_back({});
  }

  linalg::Double2d yAxis(4, -M_PI / 2, linalg::Format::Polar);

  SDL_Init(SDL_INIT_VIDEO);

  SDL_DisplayMode DM;
  SDL_GetCurrentDisplayMode(0, &DM);

  SDL_Window* window = SDL_CreateWindow("Wave generation",
                                        (DM.w - WIDTH) / 2,
                                        (DM.h - HEIGHT) / 2,
                                        WIDTH,
                                        HEIGHT,
                                        SDL_WINDOW_SHOWN);

  SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

  while (run)
  {
    uint32_t start = SDL_GetTicks();

    SDL_Event event;
    while(SDL_PollEvent(&event) != 0)
    {
      if (event.type == SDL_QUIT ||
          (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE))
      {
        run = false;
        break;
      }
      else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_UP)
      {
        for (auto& f : frequencies)
          f *= 0.01;
      }
      else if ((event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_DOWN))
      {
        for (auto& f : frequencies)
          f /= 0.01;
      }
    }

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

    linalg::Double2d vec;
    std::vector<linalg::Double2d> toDraw;
    for (int i = 0; i < signals.size(); ++i)
    {
      signals.at(i) = signals.at(i).RotateZ(-frequencies.at(i));
      vec += signals.at(i);
      auto proj = vec.Projection(yAxis);

      toDraw.push_back(vec);

#if WAVES
      points.at(i).push_back(linalg::Double2d(centerX + scale * proj.X(), centerY + scale * proj.Y()));
#else
      points.at(i).push_back(linalg::Double2d(centerX + scale * vec.X(), centerY + scale * vec.Y()));
#endif
    }

    // Draw glob of rotating lines
    SDL_RenderDrawLine(renderer, centerX, centerY, centerX + scale * toDraw.at(0).X(), centerY + scale * toDraw.at(0).Y());
    for (int i = 1; i < toDraw.size(); ++i)
      SDL_RenderDrawLine(renderer,
                          centerX + scale * toDraw.at(i - 1).X(),
                          centerY + scale * toDraw.at(i - 1).Y(),
                          centerX + scale * toDraw.at(i).X(),
                          centerY + scale * toDraw.at(i).Y());

    // Draw guide line
    auto proj = vec.Projection(yAxis);
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
    SDL_RenderDrawLine(renderer, centerX, centerY, centerX + scale * proj.X(), centerY + scale * proj.Y());
    SDL_RenderDrawLine(renderer,
                       centerX + scale * proj.X(),
                       centerY + scale * proj.Y(),
                       centerX + scale * toDraw.back().X(),
                       centerY + scale * toDraw.back().Y());

#if RENDER_ALL
    for (int i = 0; i < points.size(); ++i)
    {
      SDL_SetRenderDrawColor(renderer, colors[i].r, colors[i].g, colors[i].b, colors[i].a);
      for (auto iter = points[i].begin(); iter < points[i].end(); ++iter)
      {
        if (iter != points[i].begin())
        {
          auto prev = iter - 1;
          SDL_RenderDrawLine(renderer, iter->X(), iter->Y(), prev->X(), prev->Y());
        }

        *iter = *iter + linalg::Double2d(1, 0);
      }

      if (points[i].size() > WIDTH)
        points[i].erase(points[i].begin());
    }
#else
    SDL_SetRenderDrawColor(renderer, colors.back().r, colors.back().g, colors.back().b, colors.back().a);
    for (auto iter = points.back().begin(); iter < points.back().end(); ++iter)
    {
      if (iter != points.back().begin())
      {
        auto prev = iter - 1;
        SDL_RenderDrawLine(renderer, iter->X(), iter->Y(), prev->X(), prev->Y());
      }

      *iter = *iter + linalg::Double2d(1, 0);
    }

    if (points.back().size() > WIDTH)
      points.back().erase(points.back().begin());
#endif

    SDL_RenderPresent(renderer);
    SDL_Delay(1000 / FPS);
  }

  SDL_Quit();

  return 0;
}