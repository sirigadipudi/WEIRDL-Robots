# Course computer setup

If you are dual-booting or using a virtual machine, this describes how to set up your computer identically to how the course computers are set up.

- First, make sure you install Ubuntu 20.04.
  Other versions of Ubuntu will not work.
- Create a directory called `catkin_ws`. In the `catkin_ws` directory create another directory called `src`
- In the `src` directory clone the `fetch-picker` repository.
- Run `cd ~/catkin_ws/src/fetch-picker/course_setup/`
- Run the install script:
  ```bash
  chmod +x install_cse481c.bash
  ./install_cse481c.bash
  ```
- Run `cd ~/catkin-ws/` followed by `catkin_init_workspace` 
- Run `catkin build` to build the workspace
- Add `source ~/catkin_ws/src/fetch-picker/course_setup/labmachine.bash` at the end of `~/.bashrc`
- If you are on a laptop, then most likely you will need to edit the course tools.
  Find the function `my_ip` and change `eth0` to `wlan0`.
  Typing `source ~/.bashrc; my_ip` into the terminal should show your IP address.
  If this doesn't work, check which network interface has been assigned an IP address using `ifconfig`.

The process should take about 10-15 minutes.
If this doesn't work, let the course staff know.
