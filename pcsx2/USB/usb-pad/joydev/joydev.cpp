/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2020  PCSX2 Dev Team
 *
 *  PCSX2 is free software: you can redistribute it and/or modify it under the terms
 *  of the GNU Lesser General Public License as published by the Free Software Found-
 *  ation, either version 3 of the License, or (at your option) any later version.
 *
 *  PCSX2 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 *  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *  PURPOSE.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with PCSX2.
 *  If not, see <http://www.gnu.org/licenses/>.
 */

#include "joydev.h"
#include "../../linux/util.h"
#include "Utilities/Console.h"
#include <cassert>
#include <sstream>

namespace usb_pad
{
	namespace joydev
	{

		using namespace evdev;

#define NORM(x, n) (((uint32_t)(32768 + x) * n) / 0xFFFF)
#define NORM2(x, n) (((uint32_t)(32768 + x) * n) / 0x7FFF)

		void EnumerateDevices(device_list& list)
		{
			int fd;
			int res;
			char buf[256];

			std::stringstream str;
			struct dirent* dp;

			DIR* dirp = opendir("/dev/input/");
			if (!dirp)
			{
				Console.Warning("Error opening /dev/input/");
				return;
			}

			while ((dp = readdir(dirp)))
			{
				if (strncmp(dp->d_name, "js", 2) == 0)
				{

					str.clear();
					str.str("");
					str << "/dev/input/" << dp->d_name;
					const std::string& path = str.str();
					fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);

					if (fd < 0)
					{
						Console.Warning("Joydev: Unable to open device: %s", path.c_str());
						continue;
					}

					res = ioctl(fd, JSIOCGNAME(sizeof(buf)), buf);
					if (res < 0)
						Console.Warning("JSIOCGNAME");
					else
					{
						list.push_back({buf, buf, path});
					}

					close(fd);
				}
			}
			closedir(dirp);
		}

		int JoyDevPad::TokenIn(uint8_t* buf, int buflen)
		{
			ssize_t len;
			struct js_event events[32];
			fd_set fds;
			int maxfd;

			int range = range_max(mType);

			FD_ZERO(&fds);
			maxfd = -1;

			for (auto& device : mDevices)
			{
				FD_SET(device.cfg.fd, &fds);
				if (maxfd < device.cfg.fd)
					maxfd = device.cfg.fd;
			}

			struct timeval timeout;
			timeout.tv_usec = timeout.tv_sec = 0; // 0 - return from select immediately
			int result = select(maxfd + 1, &fds, NULL, NULL, &timeout);

			if (result <= 0)
			{
				return USB_RET_NAK; // If no new data, NAK it
			}

			for (auto& device : mDevices)
			{
				if (!FD_ISSET(device.cfg.fd, &fds))
				{
					continue;
				}

				//Non-blocking read sets len to -1 and errno to EAGAIN if no new data
				while ((len = read(device.cfg.fd, &events, sizeof(events))) > -1)
				{
					len /= sizeof(events[0]);
					for (int i = 0; i < len; i++)
					{
						js_event& event = events[i];
						if ((event.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS)
						{
							switch (device.axis_map[event.number])
							{
								case 0x80 | JOY_STEERING:
									mWheelData.steering = device.cfg.inverted[0] ? range - NORM(event.value, range) : NORM(event.value, range);
									break;
								case 0x80 | JOY_THROTTLE:
									mWheelData.throttle = device.cfg.inverted[1] ? NORM(event.value, 0xFF) : 0xFF - NORM(event.value, 0xFF);
									break;
								case 0x80 | JOY_BRAKE:
									mWheelData.brake = device.cfg.inverted[2] ? NORM(event.value, 0xFF) : 0xFF - NORM(event.value, 0xFF);
									break;

								//FIXME hatswitch mapping maybe
								case ABS_HAT0X:
								case ABS_HAT1X:
								case ABS_HAT2X:
								case ABS_HAT3X:
									if (event.value < 0) //left usually
										mWheelData.hat_horz = PAD_HAT_W;
									else if (event.value > 0) //right
										mWheelData.hat_horz = PAD_HAT_E;
									else
										mWheelData.hat_horz = PAD_HAT_COUNT;
									break;
								case ABS_HAT0Y:
								case ABS_HAT1Y:
								case ABS_HAT2Y:
								case ABS_HAT3Y:
									if (event.value < 0) //up usually
										mWheelData.hat_vert = PAD_HAT_N;
									else if (event.value > 0) //down
										mWheelData.hat_vert = PAD_HAT_S;
									else
										mWheelData.hat_vert = PAD_HAT_COUNT;
									break;
								default:
									break;
							}
						}
						else if ((event.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON)
						{
							uint16_t button = device.btn_map[event.number];
							if (button == (uint16_t)-1 || !(button & 0x8000))
								break;

							button &= ~0x8000;

							if (event.value)
								mWheelData.buttons |= 1 << convert_wt_btn(mType, button); //on
							else
								mWheelData.buttons &= ~(1 << convert_wt_btn(mType, button)); //off
						}
					}

					if (len <= 0)
					{
						break;
					}
				}
			}

			switch (mWheelData.hat_vert)
			{
				case PAD_HAT_N:
					switch (mWheelData.hat_horz)
					{
						case PAD_HAT_W:
							mWheelData.hatswitch = PAD_HAT_NW;
							break;
						case PAD_HAT_E:
							mWheelData.hatswitch = PAD_HAT_NE;
							break;
						default:
							mWheelData.hatswitch = PAD_HAT_N;
							break;
					}
					break;
				case PAD_HAT_S:
					switch (mWheelData.hat_horz)
					{
						case PAD_HAT_W:
							mWheelData.hatswitch = PAD_HAT_SW;
							break;
						case PAD_HAT_E:
							mWheelData.hatswitch = PAD_HAT_SE;
							break;
						default:
							mWheelData.hatswitch = PAD_HAT_S;
							break;
					}
					break;
				default:
					mWheelData.hatswitch = mWheelData.hat_horz;
					break;
			}

			pad_copy_data(mType, buf, mWheelData);
			return buflen;
		}

		int JoyDevPad::TokenOut(const uint8_t* data, int len)
		{
			const ff_data* ffdata = (const ff_data*)data;
			bool hires = (mType == WT_DRIVING_FORCE_PRO);
			ParseFFData(ffdata, hires);

			return len;
		}

		int JoyDevPad::Open()
		{
			device_list device_list;
			bool has_steering;
			int count;
			int32_t b_gain, gain, b_ac, ac;
			memset(&mWheelData, 0, sizeof(wheel_data_t));

			// Setting to unpressed
			mWheelData.steering = 0x3FF >> 1;
			mWheelData.clutch = 0xFF;
			mWheelData.throttle = 0xFF;
			mWheelData.brake = 0xFF;
			mWheelData.hatswitch = 0x8;
			mWheelData.hat_horz = 0x8;
			mWheelData.hat_vert = 0x8;

			mHandleFF = -1;

			std::string joypath;
			/*if (!LoadSetting(mDevType, mPort, APINAME, N_JOYSTICK, joypath))
	{
		return 1;
	}*/

			EnumerateDevices(device_list);

			for (const auto& it : device_list)
			{
				has_steering = false;
				mDevices.push_back({});

				struct device_data& device = mDevices.back();
				device.name = it.name;

				if ((device.cfg.fd = open(it.path.c_str(), O_RDWR | O_NONBLOCK)) < 0)
				{
					continue;
				}

				//int flags = fcntl(device.fd, F_GETFL, 0);
				//fcntl(device.fd, F_SETFL, flags | O_NONBLOCK);

				unsigned int version;
				if (ioctl(device.cfg.fd, JSIOCGVERSION, &version) < 0)
				{
					SysMessage("%s: Get version failed: %s\n", APINAME, strerror(errno));
					continue;
				}

				if (version < 0x010000)
				{
					SysMessage("%s: Driver version 0x%X is too old\n", APINAME, version);
					continue;
				}

				memset(device.axis_map, 0xFF, sizeof(device.axis_map));
				memset(device.btn_map, 0xFF, sizeof(device.btn_map));

				// device.cfg.controls[0..max_buttons] - mapped buttons
				// device.cfg.controls[max_buttons..etc] - mapped axes
				int max_buttons = 0;
				int max_axes = 0;
				switch (mType)
				{
					case WT_BUZZ_CONTROLLER:
						LoadBuzzMappings(mDevType, mPort, it.id, device.cfg);
						max_buttons = 20;
						break;
					case WT_KEYBOARDMANIA_CONTROLLER:
						max_buttons = 31;
						LoadMappings(mDevType, mPort, it.id, max_buttons, 0, device.cfg);
						break;
					default:
						max_buttons = JOY_STEERING;
						max_axes = 3;
						LoadMappings(mDevType, mPort, it.id, max_buttons, 3, device.cfg);
						if (!LoadSetting(mDevType, mPort, APINAME, N_GAIN_ENABLED, b_gain))
							b_gain = 1;
						if (!LoadSetting(mDevType, mPort, APINAME, N_GAIN, gain))
							gain = 100;
						if (!LoadSetting(mDevType, mPort, APINAME, N_AUTOCENTER_MANAGED, b_ac))
							b_ac = 1;
						if (!LoadSetting(mDevType, mPort, APINAME, N_AUTOCENTER, ac))
							ac = 100;
						break;
				}

				// Axis Mapping
				if (ioctl(device.cfg.fd, JSIOCGAXES, &(count)) >= 0 && count)
				{
					for (int i = 0; i < max_axes; i++)
					{
						int k = device.cfg.controls[i + max_buttons];
						if (k > -1)
						{
							device.axis_map[k] = 0x80 | (i + JOY_STEERING);
							if (k == 0)
								has_steering = true;
						}
					}
				}

				// Button Mapping
				if (ioctl(device.cfg.fd, JSIOCGBUTTONS, &(count)) >= 0 && count)
				{
					for (int i = 0; i < max_buttons; i++)
					{
						const auto k = device.cfg.controls[i];
						if (k > -1)
							device.btn_map[k] = 0x8000 | i;
					}
				}

				std::stringstream event;
				int index = 0;
				const char* tmp = it.path.c_str();
				while (*tmp && !isdigit(*tmp))
					tmp++;

				sscanf(tmp, "%d", &index);

				//TODO kernel limit is 32?
				for (int j = 0; j <= 99; j++)
				{
					event.clear();
					event.str(std::string());
					/* Try to discover the corresponding event number */
					event << "/sys/class/input/js" << index << "/device/event" << j;
					if (dir_exists(event.str()))
					{

						event.clear();
						event.str(std::string());
						event << "/dev/input/event" << j;
						break;
					}
				}

				if (!mFFdev && has_steering)
				{
					if ((mHandleFF = open(event.str().c_str(), /*O_WRONLY*/ O_RDWR)) < 0)
					{
						Console.Warning("USB: failed to open '%s'", event.str().c_str());
					}
					else
						mFFdev = new evdev::EvdevFF(mHandleFF, b_gain, gain, b_ac, ac);
				}
			}

			return 0;
		}

		int JoyDevPad::Close()
		{
			delete mFFdev;
			mFFdev = nullptr;

			if (mHandleFF != -1)
				close(mHandleFF);

			mHandleFF = -1;
			for (auto& it : mDevices)
			{
				close(it.cfg.fd);
				it.cfg.fd = -1;
			}
			return 0;
		}

	} // namespace joydev
} // namespace usb_pad
