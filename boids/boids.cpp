#include "logging.h"
#include "linalg.h"

#include <random>

#include "SDL2/SDL.h"
#include <SDL_ttf.h>

#define WIDTH 640
#define HEIGHT 480
#define FPS 60
#define SIZE 5
#define BOIDS 100

TTF_Font* font;

double ToroidalDistance (linalg::Double2d p1, linalg::Double2d p2, int width, int height)
{
  float dx = std::abs(p2.X() - p1.X());
  float dy = std::abs(p2.Y() - p1.Y());

  if (dx > width / 2)
      dx = width - dx;

  if (dy > height / 2)
      dy = height - dy;

  return std::sqrt(dx * dx + dy * dy);
}

class Boid
{
public:
  // Start with a random position in the screen
  Boid(uint32_t id)
    : mId(id)
  {
    mPos = linalg::Double2d(rand() % WIDTH, rand() % HEIGHT);

    double angle = double(rand() % 314) / 100;
    mVel = linalg::Double2d(rand() % (int)mMaxSpeed + 1, angle, linalg::Format::Polar);
  }

  uint32_t Id() const
  {
    return mId;
  }

  linalg::Double2d Position() const
  {
    return mPos;
  }

  linalg::Double2d Velocity() const
  {
    return mVel;
  }

  void UpdateMultipliers(double a, double s, double c)
  {
    aMultiplier = a;
    sMultiplier = s;
    cMultiplier = c;
  }

  linalg::Double2d Alignment(const std::vector<Boid>& boids) const
  {
    int counted = 0;
    linalg::Double2d avg;

    for (const auto& b : boids)
    {
      if (b.Id() == Id())
        continue;

      if (mPos.Distance(b.Position()) >= mRadius)
        continue;

      avg += b.Velocity();
      ++counted;
    }

    if (counted > 0)
    {
      avg /= counted;
      avg.SetMagnitude(mMaxSpeed);
      avg -= Velocity();
      avg.Limit(mMaxForce);
    }

    return avg;
  }

  linalg::Double2d Separation(const std::vector<Boid>& boids) const
  {
    int counted = 0;
    double radius = 70;
    linalg::Double2d avg;

    for (const auto& b : boids)
    {
      if (b.Id() == Id())
        continue;

      auto distance = mPos.Distance(b.Position());
      if (distance >= radius || distance < 0.01)
        continue;

      auto diff = Position() - b.Position();
      diff /= distance * distance;

      avg += diff;
      ++counted;
    }

    if (counted > 0)
    {
      avg /= counted;
      avg.SetMagnitude(mMaxSpeed);
      avg -= Velocity();
      avg.Limit(mMaxForce);
    }

    return avg;
  }

  linalg::Double2d Cohesion(const std::vector<Boid>& boids) const
  {
    int counted = 0;
    double radius = 100;
    linalg::Double2d avg;

    for (const auto& b : boids)
    {
      if (b.Id() == Id())
        continue;

      if (mPos.Distance(b.Position()) >= radius)
        continue;

      avg += b.Position();
      ++counted;
    }

    if (counted > 0)
    {
      avg /= counted;
      avg -= Position();
      avg.SetMagnitude(mMaxSpeed);
      avg -= Velocity();
      avg.Limit(mMaxForce);
    }

    return avg;
  }

  linalg::Double2d Combined(const std::vector<Boid>& boids) const
  {
    int counted = 0;
    linalg::Double2d avgAlignment, avgSeparation, avgCohesion;

    for (const auto& b : boids)
    {
      if (b.Id() == Id())
        continue;

      auto distance = ToroidalDistance(mPos, b.Position(), WIDTH, HEIGHT);  // mPos.Distance(b.Position());
      if (distance >= mRadius || distance < 0.01)
        continue;

      // Alignment
      avgAlignment += b.Velocity();

      // Separation
      auto diff = Position() - b.Position();
      diff /= distance * distance;
      avgSeparation += diff;

      // Cohesion
      avgCohesion += b.Position();

      ++counted;
    }

    if (counted > 0)
    {
      // Alignment
      avgAlignment /= counted;
      avgAlignment.SetMagnitude(mMaxSpeed);
      avgAlignment -= Velocity();
      avgAlignment.Limit(mMaxForce);

      // Separation
      avgSeparation /= counted;
      avgSeparation.SetMagnitude(mMaxSpeed);
      avgSeparation -= Velocity();
      avgSeparation.Limit(mMaxForce);

      avgCohesion /= counted;
      avgCohesion -= Position();
      avgCohesion.SetMagnitude(mMaxSpeed);
      avgCohesion -= Velocity();
      avgCohesion.Limit(mMaxForce);
    }

    return (avgAlignment * aMultiplier) + (avgSeparation * sMultiplier) + (avgCohesion * cMultiplier);
  }

  void Boundaries(linalg::Double2d& p)
  {
    if (p.X() < 0)
      p[0] = WIDTH;
    else if (p.X() > WIDTH)
      p[0] = 0;

    if (p.Y() < 0)
      p[1] = HEIGHT;
    else if (p.Y() > HEIGHT)
      p[1] = 0;
  }

  void Update(const std::vector<Boid>& boids)
  {
    linalg::Double2d acceleration = Combined(boids);

    linalg::Double2d p = mPos + mVel;

    mVel += acceleration;
    mVel.Limit(mMaxSpeed);

    Boundaries(p);

    mPos = p;
  }

  void Draw(SDL_Renderer* renderer)
  {
    auto p1 = mPos + (mVel * SIZE);
    DrawCircle(renderer, mPos.X(), mPos.Y(), SIZE);
    SDL_RenderDrawLine(renderer, mPos.X(), mPos.Y(), p1.X(), p1.Y());
  }

  void DrawCircle(SDL_Renderer* renderer, int32_t centreX, int32_t centreY, int32_t radius)
  {
    const int32_t diameter = (radius * 2);

    int32_t x = (radius - 1);
    int32_t y = 0;
    int32_t tx = 1;
    int32_t ty = 1;
    int32_t error = (tx - diameter);

    while (x >= y)
    {
        //  Each of the following renders an octant of the circle
        SDL_RenderDrawPoint(renderer, centreX + x, centreY - y);
        SDL_RenderDrawPoint(renderer, centreX + x, centreY + y);
        SDL_RenderDrawPoint(renderer, centreX - x, centreY - y);
        SDL_RenderDrawPoint(renderer, centreX - x, centreY + y);
        SDL_RenderDrawPoint(renderer, centreX + y, centreY - x);
        SDL_RenderDrawPoint(renderer, centreX + y, centreY + x);
        SDL_RenderDrawPoint(renderer, centreX - y, centreY - x);
        SDL_RenderDrawPoint(renderer, centreX - y, centreY + x);

        if (error <= 0)
        {
          ++y;
          error += ty;
          ty += 2;
        }

        if (error > 0)
        {
          --x;
          tx += 2;
          error += (tx - diameter);
        }
    }
  }

private:
  uint32_t mId;
  uint32_t mRadius = 100;

  double mMaxSpeed = 3;
  double mMaxForce = 0.2;

  double aMultiplier, sMultiplier, cMultiplier;

  linalg::Double2d mPos, mVel;
};

class Boids
{
public:
  Boids(){};

  void AddBoid()
  {
    mBoids.push_back(Boid(mBoids.size()));
  }

  void Update(double a, double s, double c)
  {
    for (auto& boid : mBoids)
    {
      boid.UpdateMultipliers(a, s, c);
      boid.Update(mBoids);
    }
  }

  void Draw(SDL_Renderer* renderer)
  {
    for (auto& boid : mBoids)
      boid.Draw(renderer);
  }

private:
  std::vector<Boid> mBoids;
};

// TODO: Move to SDL plugin library
void PrintText(SDL_Renderer* renderer, SDL_Rect dest, const std::string& text)
{
  SDL_Surface* text_surf = TTF_RenderText_Solid(font, text.c_str(), {255, 255, 255});
  SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, text_surf);

  dest.x = dest.x + (text_surf->w / 2.0f);
  dest.y = dest.y - (text_surf->h);
  dest.w = text_surf->w;
  dest.h = text_surf->h;

  SDL_RenderCopy(renderer, texture, NULL, &dest);

  SDL_DestroyTexture(texture);
  SDL_FreeSurface(text_surf);
}

class Slider
{
public:
  Slider(int x, int y, int w, int h, SDL_Color color, const std::string& text)
      : mMaxValue(h)
      , mRect{x, y, w, h}
      , mColor(color)
      , mCurrentValue(0)
      , mText(text)
  {
  }

  int Top() const
  {
    return mRect.y;
  }

  double Update(SDL_Renderer* renderer, const SDL_Event& event)
  {
    // Render borders
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderDrawRect(renderer, &mRect);

    mCurrentValue = GetNewValue();

    SDL_Rect rect = {mRect.x, Top() + mCurrentValue, mRect.w, mRect.h - mCurrentValue};

    PrintText(renderer, {mRect.x, Top(), 0, 0}, mText);
    SDL_SetRenderDrawColor(renderer, mColor.r, mColor.g, mColor.b, 200);
    SDL_RenderFillRect(renderer, &rect);

    return (double(mMaxValue - mCurrentValue) / (double)mMaxValue);
  }

  int GetNewValue()
  {
    int x, y;
    if (!(SDL_GetMouseState(&x, &y) & SDL_BUTTON_LMASK))
      return mCurrentValue;

    if ((x > mRect.x && x < mRect.x + mRect.w) && (y > mRect.y - 5 && y < mRect.y + mRect.h + 5))
      return std::min(mMaxValue, std::max(0, y - Top()));

    return mCurrentValue;
  }

private:
  const SDL_Rect mRect;
  const SDL_Color mColor;
  const int mMaxValue;
  const std::string mText;

  int mCurrentValue;
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

  Boids boids;

  bool run = true;
  bool pressed = false;

  Slider sAlignment(WIDTH - 80, HEIGHT - 210, 20, 200, {0, 255, 0}, "A");
  Slider sSeparation(WIDTH - 55, HEIGHT - 210, 20, 200, {255, 0, 0}, "S");
  Slider sCohesion(WIDTH - 30, HEIGHT - 210, 20, 200, {0, 0, 255}, "C");

  SDL_DisplayMode DM0, DM1;
  SDL_GetCurrentDisplayMode(0, &DM0);
  SDL_GetCurrentDisplayMode(1, &DM1);

  SDL_Window* window = SDL_CreateWindow("Boids",
                                        DM0.w + (DM1.w - WIDTH) / 2,
                                        (DM1.h - HEIGHT) / 2,
                                        WIDTH,
                                        HEIGHT,
                                        SDL_WINDOW_SHOWN);

  SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

  for (int i = 0; i < BOIDS; ++i)
    boids.AddBoid();

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
    boids.Draw(renderer);

    auto a = 2 * sAlignment.Update(renderer, event);
    auto s = 2 * sSeparation.Update(renderer, event);
    auto c = 2 * sCohesion.Update(renderer, event);

    boids.Update(a, s, c);

    SDL_RenderPresent(renderer);
    SDL_Delay(1000 / FPS);
  }

  TTF_Quit();
  SDL_Quit();
  return 0;
}
