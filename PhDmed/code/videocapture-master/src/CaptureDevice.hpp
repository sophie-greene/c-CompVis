/* videocapture is a tool with no special purpose
 *
 * Copyright (C) 2009 Ronny Brendel <ronnybrendel@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#ifndef CAPTURE_DEVICE_HPP
#define CAPTURE_DEVICE_HPP


#include "Prereqs.hpp"

#include <ctime>
#include <deque>
#include <linux/videodev2.h>
#include <list>
#include <mutex>
#include <string>
#include <sys/time.h>
#include <utility>

namespace std
{
    class thread;
};


/**
 * @note
 *  * changes in the most settings take effect when newly initializing the
 *    instance
 */
class CaptureDevice
{
public:

    CaptureDevice();
    CaptureDevice(const CaptureDevice&) = delete;
    CaptureDevice& operator=(const CaptureDevice&) = delete;
    ~CaptureDevice();

    /** accords with the input from init() */
    clockid_t clockId() const;
    /** set programatically */
    unsigned int bufferSize() const;
    /** possibly changed programatically */
    std::pair<unsigned int, unsigned int> captureSize() const;
    /** fixed value NONE, possibly changed programatically */
    enum v4l2_field fieldFormat() const;
    /** accords with the input from init() */
    const std::string& fileName() const;
    /** possibly changed programatically - V4L2_PIX_FMT_* values */
    __u32 pixelFormat() const;
    /** pixelFormat as a 4 character string */
    std::string pixelFormatString() const;
    /** accords with the input from init() */
    unsigned int readTimeOut() const;


    /**
     * init the camera
     *
     * @returns true on success, false on failure
     *
     * @note on failure, finish() is called implicitely
     */
    bool init(const std::string& deviceFileName,
            __u32 m_pixelFormat,
            unsigned int captureWidth,
            unsigned int captureHeight,
            unsigned int buffersCount = 2,
            clockid_t clockId = CLOCK_MONOTONIC,
            unsigned int readTimeOut = 2);

    void finish();

    void printDeviceInfo();
    void printControls();
    void printFormats();
    void printTimerInformation() const;

    struct Buffer
    {
        timespec time;
        /** if 0 -> writeable, readable; if > 0 -> readable */
        int readerCount;
        unsigned char *buffer;
    };

    /** n <= bufferCount-1 */
    std::deque<const Buffer*> lockFirstNBuffers(unsigned int n);
    void unlock(const std::deque<const Buffer*>& buffers);
    /** @returns number of newer buffers.
        when actually locking the buffer this number might defer.
        It can be larger, or it can be n-1, when previously n ~.~ */
    unsigned int newerBuffersAvailable(const timespec& newerThan);

    /**
     * blocks for several seconds
     *
     * @returns average period and standard deviation 
     */
    std::pair<double, double> determineCapturePeriod(double secondsToIterate = 5.0);

    /** creates the capture thread */
    void startCapturing();
    /** blocks until the capture thread is joined */
    void stopCapturing();
    bool isCapturing() const;

    void pauseCapturing(bool pause);

    /** @see http://www.linuxtv.org/downloads/video4linux/API/V4L2_API/spec-single/v4l2.html#V4L2-QUERYCTRL
        @see http://www.linuxtv.org/downloads/video4linux/API/V4L2_API/spec-single/v4l2.html#V4L2-QUERYMENU */
    std::pair<std::list<struct v4l2_queryctrl>, std::list<struct v4l2_querymenu> > controls();

    /** @see http://www.linuxtv.org/downloads/video4linux/API/V4L2_API/spec-single/v4l2.html#V4L2-CONTROL
        @returns whether the result is valid - why? -> better error checking to come */
    bool control(struct v4l2_control&);
    /** @returns whether the call succeeded - why not ? -> better error checking to come */
    bool setControl(const struct v4l2_control&);

private:

    bool queryControl(struct v4l2_queryctrl&);
    std::list<struct v4l2_querymenu> menus(const struct v4l2_queryctrl&);

    static void captureThread(CaptureDevice* camera);
    static void determineCapturePeriodThread(double, CaptureDevice*,
            std::pair<double,double>*);

    int xv4l2_ioctl(int fileDescriptor, int request, void *arg);

    unsigned int m_captureHeight;
    unsigned int m_captureWidth;
    int m_fileDescriptor;
    enum v4l2_field m_fieldFormat;
    std::string m_deviceFileName;
    __u32 m_pixelFormat;
    unsigned int m_readTimeOut;

    unsigned int m_bufferSize;
    std::list<Buffer> m_buffers;
    std::deque<Buffer*> m_timelySortedBuffers;
    std::mutex m_timelySortedBuffersMutex;

    clockid_t m_timerClockId;
    struct timespec m_timerResolution;
    struct timespec m_timerStart;
    struct timeval m_realStartTime;

    std::thread *m_captureThread;
    bool m_captureThreadCancellationFlag;

    std::mutex m_fileAccessMutex;
    std::mutex m_pauseCapturingMutex;
};


#endif /* CAPTURE_DEVICE_HPP */

