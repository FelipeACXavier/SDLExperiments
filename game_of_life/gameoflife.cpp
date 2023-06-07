#include "logging.h"
#include "linalg.h"

#include <algorithm>
#include <memory>
#include <random>

#include "SDL2/SDL.h"
#include <SDL_ttf.h>

#define WIDTH 1000
#define HEIGHT 1000
#define TILE_SIZE 10
#define FPS 15
#define ALIVE_PROB 50

TTF_Font* font;

std::vector<linalg::Int2d> gliderR(int x, int y)
{
  std::vector<linalg::Int2d> coordinates;
  coordinates.push_back({x, y});
  coordinates.push_back({x - 1, y});
  coordinates.push_back({x, y + 1});
  coordinates.push_back({x + 1, y - 1});
  coordinates.push_back({x + 1, y + 1});
  return coordinates;
}

std::vector<linalg::Int2d> gliderL(int x, int y)
{
  std::vector<linalg::Int2d> coordinates;
  coordinates.push_back({x, y});
  coordinates.push_back({x, y + 1});
  coordinates.push_back({x - 1, y});
  coordinates.push_back({x + 1, y - 1});
  coordinates.push_back({x + 1, y + 1});
  return coordinates;
}

void PrintText(SDL_Renderer* renderer, SDL_Rect dest, const std::string& text)
{
  SDL_Surface* text_surf = TTF_RenderText_Solid(font, text.c_str(), {255, 255, 255});
  SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, text_surf);

  dest.x = dest.x + 15;
  dest.y = dest.y + 15;
  dest.w = 20;
  dest.h = 20;

  SDL_RenderCopy(renderer, texture, NULL, &dest);

  SDL_DestroyTexture(texture);
  SDL_FreeSurface(text_surf);
}

class Map
{
public:
  Map(uint16_t width, uint16_t height)
    : mWidth(width)
    , mHeight(height)
  {}

  void Start(const std::vector<linalg::Int2d>& initial)
  {
    for (uint16_t j = 0; j < mHeight; ++j)
    {
      for (uint16_t i = 0; i < mWidth; ++i)
      {
        if (initial.empty())
          mCells.push_back((rand() % 100) > ALIVE_PROB);
        else
          mCells.push_back(std::find_if(initial.begin(), initial.end(), [i, j](const linalg::Int2d& p){ return p == linalg::Int2d(i, j); }) != initial.end());
      }
    }
  }

  uint8_t Count(const std::vector<bool>& map, uint16_t x, uint16_t y)
  {
    uint8_t alive = 0;
    for (int dy = -1; dy <= 1; ++dy)
    {
      for (int dx = -1; dx <= 1; ++dx)
      {
        if ((dx == 0 && dy == 0) ||
            uint16_t(x + dx) > (mWidth - 1) ||
            uint16_t(y + dy) > (mHeight - 1))
          continue;

        alive += uint8_t(map.at((y + dy) * mWidth + (x + dx)));
      }
    }

    return alive;
  }

  void Update()
  {
    const std::vector<bool> tmp = mCells;
    for (int j = 0; j < mHeight; ++j)
    {
      for (int i = 0; i < mWidth; ++i)
      {
        auto alive = Count(tmp, i, j);

        if (mCells.at(j * mWidth + i))
        {
          if (alive == 2 || alive == 3)
            mCells.at(j * mWidth + i) = true;
          else
            mCells.at(j * mWidth + i) = false;
        }
        else
        {
          if (alive == 3)
            mCells.at(j * mWidth + i) = true;
        }
      }
    }
  }

  void Draw(SDL_Renderer* renderer)
  {
    for (int j = 0; j < mHeight; ++j)
    {
      for (int i = 0; i < mWidth; ++i)
      {
        SDL_Rect rect{i * TILE_SIZE, j * TILE_SIZE, TILE_SIZE, TILE_SIZE};

        if (mCells.at(j * mWidth + i))
          SDL_RenderDrawRect(renderer, &rect);
      }
    }
  }

private:
  uint16_t mWidth;
  uint16_t mHeight;

  std::vector<bool> mCells;
};

int main()
{
  srand(time(NULL));

  SDL_Init(SDL_INIT_VIDEO);
  TTF_Init();

#ifdef FONT_FILE
  font = TTF_OpenFont(FONT_FILE, 18);
#else
  LOG_ERROR("Failed to load font");
  return -1;
#endif

  bool run = true;

  SDL_DisplayMode DM0, DM1;
  SDL_GetCurrentDisplayMode(0, &DM0);
  SDL_GetCurrentDisplayMode(1, &DM1);

  SDL_Window* window = SDL_CreateWindow("Game of life",
                                        DM0.w + (DM1.w - WIDTH) / 2,
                                        (DM1.h - HEIGHT) / 2,
                                        WIDTH,
                                        HEIGHT,
                                        SDL_WINDOW_SHOWN);

  SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

  Map map(WIDTH / TILE_SIZE, HEIGHT / TILE_SIZE);
  map.Start({});

  while (run)
  {
    SDL_Event event;
    while (SDL_PollEvent(&event) != 0)
    {
      if (event.type == SDL_QUIT || (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE))
      {
        run = false;
        break;
      }
    }

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    SDL_RenderClear(renderer);

    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 200);

    map.Update();
    map.Draw(renderer);

    SDL_RenderPresent(renderer);
    SDL_Delay(1000 / FPS);
  }

  TTF_Quit();
  SDL_Quit();

  return 0;
}
