#include "uro_shared_memory/CameraImage.hh"
#include <fcntl.h>
#include <iostream>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/sem.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

using namespace uro_shared_memory;

namespace
{
    // 共有メモリ名
    const char* CAMERA_IMAGE_SHM_NAME = "/uro_camera_image_shm";
    // セマフォ名
    const char* CAMERA_IMAGE_SEM_NAME = "/uro_camera_image_sem";
} // namespace

CameraImage::CameraImage(bool master) : mMaster(master)
{
    if (mMaster)
    {
        // 共有メモリをO_CREATフラグで作成（既に存在してもエラーにならない）
        mFd = shm_open(CAMERA_IMAGE_SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (mFd < 0)
        {
            std::cerr << "Failed to create or open shared memory." << std::endl;
            return;
        }
        // 共有メモリのサイズを設定
        if (ftruncate(mFd, sizeof(CameraImageData)) == -1)
        {
            std::cerr << "Failed to set shared memory size." << std::endl;
            close(mFd);
            mFd = 0;
            return;
        }
        // セマフォを作成
        mpSem = sem_open(CAMERA_IMAGE_SEM_NAME, O_CREAT | O_RDWR, 0666, 1);
        if (mpSem == SEM_FAILED)
        {
            std::cerr << "Failed to create or open camera image semaphore." << std::endl;
            close(mFd);
            return;
        }
        // 共有メモリをプロセスのアドレス空間にマップ
        mpCameraImageData = static_cast<CameraImageData*>(
            mmap(nullptr, sizeof(CameraImageData), PROT_READ | PROT_WRITE, MAP_SHARED, mFd, 0));
        if (mpCameraImageData == MAP_FAILED)
        {
            std::cerr << "Failed to map shared memory." << std::endl;
            sem_close(mpSem);
            close(mFd);
            return;
        }
    }
}
CameraImage::~CameraImage()
{
    if (mMaster)
    {
    }
}
int
CameraImage::SetImage(int aWidth, int aHeight, int aChannesls, const unsigned char* pImage)
{
    return 0;
}
int
CameraImage::GetImage(int& rWidth, int& rHeight, int& rChannesls, unsigned char* pImage)
{
    return 0;
}
int
CameraImage::SetSerial(long aSeral)
{
    return 0;
}
int
CameraImage::GetSerial(long& rSeral)
{
    return 0;
}
int
CameraImage::SetRecording(bool aRecording)
{
    return 0;
}
int
CameraImage::GetRecording(bool& rRecording)
{
    return 0;
}
void
CameraImage::Lock()
{
    for (;;)
    {
        if (sem_wait(mpSem) == 0)
        {
            return;
        }
        if (errno != EINTR)
        {
            perror("sem_wait");
        }
    }
}
void
CameraImage::Unlock()
{
    if (sem_post(mpSem) < 0)
    {
        perror("sem_post");
    }
}
