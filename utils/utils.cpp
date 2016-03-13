
#include "utils.h"
#include <errno.h>
#include <dirent.h>
#include <string.h>
#include <stdio.h>
#include "logger.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


namespace robo {

uint64_t soundspeed_usec2cm(uint64_t usec, float temp)
{
    // TODO: adjust for temperature!
    // The speed of sound is 340 m/s or 29 microseconds per centimeter.
    // The ping travels out and back, so to find the distance of the
    // object we take half of the distance travelled.
    // http://www.parallax.com/sites/default/files/downloads/28015-PING-Documentation-v1.6.pdf
    const float speed = 331.5f + (0.6f * temp); // m/sec
    return usec / (10000.0f / speed);
}

int write_int_to_file(const char *filename, int value)
{
    char buf[64];
    const int len = snprintf(buf, sizeof(buf), "%d", value);
    return write_buf_to_file(filename, buf, len);
}

int open_path(const char *dir1, const char *dir2, int &fd, int flags)
{
    char path[256];
    snprintf(path, sizeof(path), "%s/%s", dir1, dir2);
    fd = HANDLE_EINTR(::open(path, flags));
    if (fd == -1)
        return errno;
    return 0;
}

int write_buf_to_path(const char *dir1, const char *dir2, const char *buf, int bufLen)
{
    int fd = -1;
    int ret = open_path(dir1, dir2, fd, O_WRONLY);
    if (ret)
        return ret;

    ret = HANDLE_EINTR(::write(fd, buf, bufLen));
    if (ret == -1)
        ret = errno;
    else if (ret != bufLen)
        ret = EFBIG;
    else
        ret = 0;
    close(fd);
    return ret;
}

int write_buf_to_file(const char *filename, const char *buf, int bufLen)
{
    const int fd = HANDLE_EINTR(::open(filename, O_WRONLY));
    if (fd < 0)
        return errno;

    int ret = HANDLE_EINTR(::write(fd, buf, bufLen));
    if (ret == -1)
        ret = errno;
    else if (ret != bufLen)
        ret = EFBIG;
    else
        ret = 0;
    close(fd);
    return ret;
}

int build_sys_path(const char *partial_path, const char *prefix, char *full_path, size_t full_path_len)
{
    logger(LOG_INFO, "building sys path for dir=%s prefix=%s", partial_path, prefix);

    int ret = EBADF;
    DIR *dp = ::opendir(partial_path);
    if (dp == NULL) 
        return errno;

    while (1)
    {
        errno = 0;
        struct dirent *ep = ::readdir(dp);
        if (ep == NULL)
        {
            if (errno)
                ret = errno;
            break;
        }

        // Enforce that the prefix must be the first part of the file
        const char *found_string = strstr(ep->d_name, prefix);
        if (found_string != NULL && (ep->d_name - found_string) == 0) 
        {
            snprintf(full_path, full_path_len, "%s/%s", partial_path, ep->d_name);
            ret = 0;
            break;
        }
    }

    ::closedir(dp);
    return ret;
}

static int get_capemgr_slot_file(FILE *&file)
{
    char ctrl_dir[256] = { 0 };
    int ret = build_sys_path("/sys/devices", "bone_capemgr", ctrl_dir, sizeof(ctrl_dir));
    if (ret)
        return ret;

    char *end = ctrl_dir + strlen(ctrl_dir);
    if ((end - ctrl_dir) >= sizeof(ctrl_dir))
        return EFAULT;

    snprintf(end, sizeof(ctrl_dir) - (end - ctrl_dir), "/slots");

    file = fopen(ctrl_dir, "r+");
    if (!file) 
        return errno;
    return 0;
}

int activate_cape_mgr_slot(const char *name)
{
    logger(LOG_INFO, "activating capemgr slot name=%s", name);

    FILE *file = 0;
    int ret = get_capemgr_slot_file(file);
    if (ret) 
        return ret;

    bool isLoaded = false;
    char line[256];
    while (fgets(line, sizeof(line), file)) 
    {
        //the device is already loaded?
        if (strstr(line, name)) 
        {
            isLoaded = true;
            break;
        }
    }

    //if the device isn't already loaded, load it, and return
    if (!isLoaded)
        fprintf(file, "%s", name);
    fclose(file);
    return 0;
}

int deactivate_cape_mgr_slot(const char *name)
{
    logger(LOG_INFO, "de-activating capemgr slot name=%s", name);

    FILE *file = 0;
    int ret = get_capemgr_slot_file(file);
    if (ret) 
        return ret;

    char line[256];
    while (fgets(line, sizeof(line), file)) 
    {
        //the device is loaded, let's unload it
        if (strstr(line, name)) 
        {
            const char *slot_line = strtok(line, ":");
            //remove leading spaces
            while(*slot_line == ' ')
                slot_line++;

            fprintf(file, "-%s", slot_line);
            break;
        }
    }

    //not loaded, close file
    fclose(file);
    return 0;
}


} // namespace robo