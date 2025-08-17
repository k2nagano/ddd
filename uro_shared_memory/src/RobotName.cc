#include "uro_shared_memory/RobotName.hh"
#include <cstring>
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
    const char* ROBOT_NAME_SHM_NAME = "/uro_robot_name_shm";
    // セマフォ名
    const char* ROBOT_NAME_SEM_NAME = "/uro_robot_name_sem";
    // RobotNameSize
    const size_t ROBOT_NAME_SIZE = 8;
} // namespace

RobotName::RobotName(bool master) : mMaster(master)
{
    if (mMaster)
    {
        // 共有メモリをO_CREATフラグで作成（既に存在してもエラーにならない）
        mFd = shm_open(ROBOT_NAME_SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (mFd < 0)
        {
            std::cerr << "Failed to create or open shared memory." << std::endl;
            return;
        }
        // 共有メモリのサイズを設定
        if (ftruncate(mFd, ROBOT_NAME_SIZE) == -1)
        {
            std::cerr << "Failed to set shared memory size." << std::endl;
            close(mFd);
            mFd = 0;
            return;
        }
        // セマフォを作成
        mpSem = sem_open(ROBOT_NAME_SEM_NAME, O_CREAT | O_RDWR, 0666, 1);
        if (mpSem == SEM_FAILED)
        {
            std::cerr << "Failed to create or open camera image semaphore." << std::endl;
            close(mFd);
            return;
        }
        // 共有メモリをプロセスのアドレス空間にマップ
        mpRobotName = static_cast<unsigned char*>(
            mmap(nullptr, ROBOT_NAME_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mFd, 0));
        if (mpRobotName == MAP_FAILED)
        {
            std::cerr << "Failed to map shared memory." << std::endl;
            sem_close(mpSem);
            close(mFd);
            return;
        }
    }
}
RobotName::~RobotName()
{
    if (mMaster)
    {
    }
}
int
RobotName::Set(const std::string& aRobotName)
{
    // 入力文字列の長さをチェック
    // 8文字を超えている場合はエラーとする（ヌル終端文字'\0'を含む）
    if (aRobotName.length() >= sizeof(mpRobotName))
    {
        // -1などのエラーコードを返す
        return -1;
    }

    Lock();
    // 文字列をmpRobotNameにコピー
    // strncpyは、バッファオーバーフローを防止するために安全な方法です。
    // ただし、終端にヌル文字が追加されない場合があるので、手動で追加します。
    std::strncpy(reinterpret_cast<char*>(mpRobotName), aRobotName.c_str(), sizeof(mpRobotName) - 1);
    mpRobotName[sizeof(mpRobotName) - 1] = '\0'; // ヌル終端文字を必ず追加
    Unlock();
    return 0;
}
int
RobotName::Get(std::string& rRobotName)
{
    Lock();
    // mpRobotNameに格納されている文字列をstd::stringにコピー
    // c_str()を使用してconst char*に変換してから、std::stringに代入します。
    rRobotName = reinterpret_cast<const char*>(mpRobotName);
    Unlock();
    return 0;
}
void
RobotName::Lock()
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
RobotName::Unlock()
{
    if (sem_post(mpSem) < 0)
    {
        perror("sem_post");
    }
}
