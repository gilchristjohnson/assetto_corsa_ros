# Assetto Corsa ROS

This repository integrates Assetto Corsa with ROS 2. 

## Installation

### Clone and prepare your workspace
1. Navigate to the `src` directory of your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   ```
2. Create a folder to hold the simulation packages and enter it:
   ```bash
   mkdir -p simulation
   cd simulation
   ```
3. Clone this repository into the `simulation` directory:
   ```bash
   git clone git@github.com:cast-racing/assetto_corsa_ros.git
   cd assetto_corsa_ros
   ```
4. Run the setup script to install dependencies and configure the environment:
   ```bash
   ./assetto_corsa_ros_setup.sh
   ```

### Responding to setup prompts
The setup script is interactive so you can control what gets installed or replaced. During the run you may be asked to:

- Stop a running Assetto Corsa instance: answer `y` when you see `Assetto Corsa is running. Stop Assetto Corsa to proceed?` so the script can safely continue.
- Remove conflicting temporary folders: the script offers to move any existing `temp/` folder to the trash (`Move "temp/" to trash?`). Accepting prevents the installation from failing.
- Confirm the discovered installation path with `Is that the right installation?`. Choose `y` when the detected path matches your Assetto Corsa library.
- Decide whether to install Proton-GE 9-20 (`Install ProtonGE 9-20?` or `Reinstall ProtonGE 9-20?`). Choose `y` to install the tested Proton build the script relies on.
- Install or reinstall Content Manager (`Install Content Manager?` / `Reinstall Content Manager?`) and its supporting fonts.
- Install or upgrade Custom Shaders Patch (`Install CSP (Custom Shaders Patch) v0.2.11?` and related prompts). Accepting keeps the plugin features aligned with this ROS stack.
- Deploy the ROS assets inside Assetto Corsa (`Install/Reinstall Assetto Corsa ROS Control Preset?`, `Install/Reinstall Assetto Corsa ROS Plugin?`, and `Install/Reinstall Assetto Corsa ROS Vehicle Content?`). Answer `y` to link the provided control presets, plugin, and vehicle content into your game directory.

If a prompt does not apply to your setup—such as reinstalling existing components—you can answer `n` to skip that action and keep your current files.

### Selecting the Proton-GE version in Steam
After the script installs Proton-GE it now pauses so you can configure Steam before continuing. When prompted:
1. Restart Steam.
2. In the Steam Library, right-click **Assetto Corsa** and choose **Properties**.
3. Open the **Compatibility** tab.
4. Enable **Force the use of a specific Steam Play compatibility tool**.
5. Choose **Proton-GE 9-20** from the drop-down list.
6. Return to the setup script and confirm that you completed the steps.

Once the setup script finishes and the Proton version is selected, build and source your ROS 2 workspace as usual.
