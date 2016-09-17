#ifndef _PING_UTILS_HXX_
#define _PING_UTILS_HXX_

struct ThreadArg
{
    int fd;
};

template <typename ptr_type, typename fn_type>
bool rw_repeated(int fd, ptr_type bufptr, size_t len, long long *time_first,
    fn_type fn, const char *fnname)
{
    uint8_t *buf = (uint8_t *)bufptr;
    if (time_first)
    {
        *time_first = 0;
    }
    while (len > 0)
    {
        int ret = fn(fd, buf, len);
        if (ret > 0)
        {
            len -= ret;
            buf += ret;
            if (time_first && !*time_first)
            {
                *time_first = os_get_time_monotonic();
            }
        }
        else if (ret == 0)
        {
            fprintf(stderr, "EOF in %s\n", fnname);
            ::close(fd);
            return false;
        }
        else if (errno == EAGAIN)
        {
            continue;
        }
        else
        {
            fprintf(
                stderr, "Error in %s: %d %s\n", fnname, errno, strerror(errno));
            ::close(fd);
            return false;
        }
    }
    return true;
}

/// Prints a time difference statistic
///
/// @param name user-readable text on what this stat is about
/// @param first begin timestamp nanoseconds
/// @param second end timestamp nanoseconds
///
void printstat(const char *name, long long first, long long second)
{
    printf("Time(%30s) = %10d usec\n", name, (int)((second - first) / 1000));
}

#endif // _PING_UTILS_HXX_
