#if !defined(URO_SHARED_MEMORY_CAMERA_IMAGE_HH)
#define URO_SHARED_MEMORY_CAMERA_IMAGE_HH

#include <semaphore.h>
#include <string>

namespace uro_shared_memory
{
    // カメラ画像データを格納する構造体
    struct CameraImageData
    {
        static constexpr int MAX_WIDTH = 1920;
        static constexpr int MAX_HEIGHT = 1080;
        static constexpr int MAX_CHANNELS = 3;
        static constexpr int MAX_DATE_TIME = 24;

        int mWidth;
        int mHeight;
        int mChannels;
        unsigned char mImage[MAX_WIDTH * MAX_HEIGHT * MAX_CHANNELS];
        long mSerial;
        bool mIsRecording;
        unsigned char mDateTime[MAX_DATE_TIME]; // "yyyy-mm-dd HH:MM:SS"
        double mTimestamp;
        double mX;
        double mY;
        double mZ;
        double mYaw;
    };

    class CameraImage
    {
    public:
        explicit CameraImage(bool master = false);
        ~CameraImage();

        int SetImage(int aWidth, int aHeight, int aChannesls, const unsigned char* pImage);
        int GetImage(int& rWidth, int& rHeight, int& rChannesls, unsigned char* pImage);
        int SetSerial(long aSeral);
        int GetSerial(long& rSeral);
        int SetRecording(bool aRecording);
        int GetRecording(bool& rRecording);

    private:
        void Lock();
        void Unlock();

    private:
        int mFd;
        sem_t* mpSem;
        CameraImageData* mpCameraImageData;
        bool mMaster;
    };
} // namespace uro_shared_memory

#endif // #if !defined(URO_SHARED_MEMORY_CAMERA_IMAGE_HH)
