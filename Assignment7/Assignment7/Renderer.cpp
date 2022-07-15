//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <thread>
#include <condition_variable>
#include <mutex>

inline float deg2rad(const float &deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00016f;

std::condition_variable cv;
std::mutex thTex;
int numThread = 0;
std::mutex mutex;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene &scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    int spp = 1024;
    std::cout << "SPP: " << spp << "\n";

    auto castThread = [&](int m, float x, float y)
    {
        {
            std::lock_guard lg(thTex);
            numThread++;
        }
        static const float dx[] = {1, 1, -1, -1};
        const float Dx = (0.5 / (float)scene.width) * imageAspectRatio * scale;
        static const float dy[] = {1, -1, 1, -1};
        const float Dy = (0.5 / (float)scene.height) * scale;
        for (int i = 0; i < 4; i++)
        {
            Vector3f sum;
            for (int k = 0; k < spp; k++)
            {
                Vector3f dir = normalize(Vector3f(-x + dx[i] * Dx, y + dy[i] * Dy, 1));
                sum += scene.castRay(Ray(eye_pos, dir), 0) / spp;
            }
            framebuffer[m] += sum / 4;
        }
        {
            std::lock_guard lg(thTex);
            numThread--;
        }
        cv.notify_one();
    };

    for (uint32_t j = 0; j < scene.height; ++j)
    {
        for (uint32_t i = 0; i < scene.width; ++i)
        {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

            {
                std::unique_lock<std::mutex> lock(mutex);
                cv.wait(lock, []
                        { return numThread < 12; });
            }
            std::thread th(castThread, m, x, y);
            th.detach();

            m++;
        }
        UpdateProgress(j / (float)scene.height);
    }

    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, []
                { return numThread == 0; });
    }

    UpdateProgress(1.f);

    // save framebuffer to file
    FILE *fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i)
    {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
