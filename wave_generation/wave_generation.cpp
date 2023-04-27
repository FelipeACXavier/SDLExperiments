#include <iostream>
#include <vector>
#include <random>

#include "linalg.h"

#include <SDL2/SDL.h>

#define FPS 60
#define WIDTH 1000
#define HEIGHT 600

int main()
{
  srand(time(NULL));

  std::vector<double> frequencies;
  std::vector<linalg::Vector<double>> points;
  std::vector<linalg::Vector<double>> signals;

  bool run = true;

  int waves = 10;
  int scale = 10;
  int centerX = WIDTH / 8;
  int centerY = HEIGHT / 2;
  double phase = 0;

  for (int i = 1; i < waves + 1; i++)
  {
    signals.push_back(linalg::Vector<double>(i, phase, linalg::Format::Polar));

    // Start with random phase and frequency
    frequencies.push_back(double(rand() % 100 + 1) / 1000);
    phase += M_PI / (rand() % 10 + 1);
  }

  linalg::Vector<double> b(4, -M_PI / 2, linalg::Format::Polar);

  SDL_Init(SDL_INIT_VIDEO);

  SDL_DisplayMode DM;
  SDL_GetCurrentDisplayMode(0, &DM);

  SDL_Window* window = SDL_CreateWindow("Wave generation", (DM.w - WIDTH) / 2, (DM.h - HEIGHT) / 2, WIDTH, HEIGHT, SDL_WINDOW_SHOWN);
  SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

  while (run)
  {
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
          f += 0.01;
      }
      else if ((event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_DOWN))
      {
        for (auto& f : frequencies)
          f -= 0.01;
      }
    }

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

    linalg::Vector<double> vec(2);
    for (int i = 0; i < signals.size(); ++i)
    {
      signals.at(i) = signals.at(i).RotateZ(-frequencies.at(i));
      vec = vec + signals.at(i);
    }

    for (const auto& signal : signals)
      SDL_RenderDrawLine(renderer, centerX, centerY, centerX + scale * signal.X(), centerY + scale * signal.Y());

    points.push_back(linalg::Vector<double>(centerX + scale * signals.back().Mag(), centerY + scale * vec.Y()));

    auto proj = vec.Projection(b);
    SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
    SDL_RenderDrawLine(renderer, centerX, centerY, centerX + scale * proj.X(), centerY + scale * proj.Y());
    SDL_RenderDrawLine(renderer,
                       centerX + scale * proj.X(),
                       centerY + scale * proj.Y(),
                       centerX + scale * signals.back().Mag(),
                       centerY + scale * vec.Y());

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    for (auto iter = points.begin(); iter < points.end();)
    {
      if (iter != points.begin())
      {
        auto prev = iter - 1;
        SDL_RenderDrawLine(renderer, iter->X(), iter->Y(), prev->X(), prev->Y());
      }

      *iter = *iter + linalg::Vector<double>(1, 0);
      if (iter->X() >= WIDTH + 5)
        iter = points.erase(iter);
      else
        iter++;
    }

    SDL_RenderPresent(renderer);
    SDL_Delay(1000 / FPS);
  }

  SDL_Quit();

  return 0;
}