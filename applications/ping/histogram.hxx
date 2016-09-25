#ifndef _PING_HISTO_HXX_
#define _PING_HISTO_HXX_

#include <stdio.h>
#include <string>

class Histogram
{
public:
    void add(int value)
    {
        ++totalCount_;
        counter_[value]++;
    }

    void print()
    {
        if (counter_.empty())
        {
            printf("histogram: no entries\n");
            return;
        }
        printf("histogram: %d samples; quartiles [%d, %d, %d]\n", count(),
            percentile(25), percentile(50), percentile(75));
        int p20 = percentile(20);
        int p80 = percentile(80);
        int diff = p80 - p20;
        diff /= 3;
        int split = 1;
        while (diff / 10 > 0)
        {
            diff /= 10;
            split *= 10;
        }
        diff *= split;

        int last = percentile(100);
        int nextbucket = (percentile(0) / diff) * diff;
        float cumulative = 0.0;
        bool skipping = false;
        while (nextbucket <= last)
        {
            int cnt = countbucket(nextbucket, diff);
            if (!skipping && !cnt && (cumulative >= 70) &&
                (countbucket(nextbucket + diff, diff) == 0))
            {
                skipping = true;
                printf("//\n");
            }
            if (cnt)
            {
                skipping = false;
            }
            float percent = cnt * 100.0 / count();
            cumulative += percent;
            if (!skipping)
            {
                string line((size_t)(percent / 3), '=');
                printf("%4.1f%% %5.1f%%%7d %3d [%s\n", percent, cumulative,
                    nextbucket, cnt, line.c_str());
            }
            nextbucket += diff;
        }
    }

private:
    int count() {
        return totalCount_;
    }
    
    int countbucket(int bucket_start, int bucket_len)
    {
        auto it = counter_.upper_bound(bucket_start - 1);
        int cnt = 0;
        while (it != counter_.end() && it->first < (bucket_start + bucket_len))
        {
            cnt += it->second;
            ++it;
        }
        return cnt;
    }

    int percentile(int p)
    {
        if (counter_.empty())
            return 0;
        if (p >= 100)
        {
            return (--counter_.end())->first;
        }
        if (p <= 0)
        {
            return counter_.begin()->first;
        }
        int step = p * count() / 100;
        auto it = counter_.begin();
        while (step > 0 && it != counter_.end()) {
            step -= it->second;
            ++it;
        }
        return it->first;
    }

    size_t totalCount_{0};
    std::map<int, int> counter_;
};

#endif // _PING_HISTO_HXX_
