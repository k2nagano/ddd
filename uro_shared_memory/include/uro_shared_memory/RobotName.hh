#if !defined(URO_SHARED_MEMORY_ROBOT_NAME_HH)
#define URO_SHARED_MEMORY_ROBOT_NAME_HH

#include <semaphore.h>
#include <string>

namespace uro_shared_memory
{
    class RobotName
    {
    public:
        explicit RobotName(bool master = false);
        ~RobotName();

        int Set(const std::string& aRobotName);
        int Get(std::string& rRobotName);

    private:
        void Lock();
        void Unlock();

    private:
        int mFd;
        sem_t* mpSem;
        unsigned char* mpRobotName;
        bool mMaster;
    };
} // namespace uro_shared_memory

#endif // #if !defined(URO_SHARED_MEMORY_ROBOT_NAME_HH)
