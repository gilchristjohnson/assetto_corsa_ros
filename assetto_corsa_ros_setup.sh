#!/bin/bash
# Preventing from running as root
if [[ $USER == "root" ]]; then
  echo "Please do not run as root."
  exit 1
fi

# Useful variables
GE_version="9-20"; CSP_version="0.2.11"
declare -a COMPAT_TOOLS_DIRS=()
declare -a STEAM_DIRS=()
## Defining text styles for readablity
bold=$(tput bold); normal=$(tput sgr0)
## Required native packages
req_packages=("wget" "tar" "unzip" "glib2" "steam" "protontricks")

# Setting paths
function set_paths_for {
  # Setting steam paths
  if [[ $1 == "steam" ]]; then
    LOCAL="$HOME/.local"
    APPLAUNCH_AC="steam -applaunch 244210 %u"
    STEAM_LIBRARY_VDF="${HOME}/.steam/steam/steamapps/libraryfolders.vdf"

    # Discover possible Steam directories
    local -a candidates=(
      "$HOME/.local/share/Steam"
      "$HOME/.steam/steam"
      "$HOME/.steam/root"
    )

    STEAM_DIRS=()
    for candidate in "${candidates[@]}"; do
      if [[ -d "$candidate" ]]; then
        STEAM_DIRS+=("$candidate")
      fi
    done

    # Fall back to the default path if none of the candidates currently exist
    if [[ ${#STEAM_DIRS[@]} -eq 0 ]]; then
      STEAM_DIRS+=("$HOME/.local/share/Steam")
    fi

    COMPAT_TOOLS_DIRS=()
    for steam_dir in "${STEAM_DIRS[@]}"; do
      COMPAT_TOOLS_DIRS+=("$steam_dir/compatibilitytools.d")
    done

    # Backwards compatible variables for the rest of the script
    COMPAT_TOOLS_DIR="${COMPAT_TOOLS_DIRS[0]}"

    # Setting universal paths
    AC_COMMON="${STEAM_DIRS[0]}/steamapps/common/assettocorsa"
    AC_DESKTOP="$LOCAL/share/applications/Assetto Corsa.desktop"
  # Setting AC paths
  elif [[ $1 == "assettocorsa" ]]; then
    AC_COMMON="$2"
    STEAMAPPS="${AC_COMMON%"/common/assettocorsa"}"
    AC_COMPATDATA="$STEAMAPPS/compatdata/244210"
  else
    Error "set_paths_for: '$1' is not a valid option"
  fi
}

# Checking OS compatability
function get_release {
  os_release="$(cat /etc/os-release)"
  echo "$(echo $os_release | sed "s/.* $1=//g" | sed "s/$1=\"//g" |  sed "s/ .*//g" | sed "s/\"//g")"
}
function CheckOS {
  OS="$(get_release ID)"; VERSION="$(get_release VERSION_ID)"; OS_name="$(get_release PRETTY_NAME)"
  if [[ "$OS" != "ubuntu" || "$VERSION" != "22.04" ]]; then
    echo "Detected ${OS_name:-$OS $VERSION}. This script is only compatible with Ubuntu 22.04."
    exit 1
  fi

  pm_install="apt install"
  pm_list="apt list --installed"
}

function CheckUnsupportedSteamInstallations {
  if command -v flatpak > /dev/null 2>&1; then
    if flatpak list --app --columns=application 2> /dev/null | grep -Fxq "com.valvesoftware.Steam"; then
      echo "Flatpak installation of Steam detected. This script is not compatible with the Flatpak version of Steam."
      echo "Please remove it by running: ${bold}flatpak uninstall --delete-data com.valvesoftware.Steam${normal}."
      exit 1
    fi
  fi

  if command -v snap > /dev/null 2>&1; then
    if snap list steam > /dev/null 2>&1; then
      echo "Snap installation of Steam detected. This script is not compatible with the Snap version of Steam."
      echo "Please remove it by running: ${bold}sudo snap remove steam --purge${normal}."
      exit 1
    fi
  fi
}

# Checking if required packages are installed
function CheckDependencies {
  installed_packages=($($pm_list))
  for package in ${req_packages[@]}; do
    if [[ $package == "steam" ]]; then
      if [[ ${installed_packages[@]} != *"steam"* ]]; then
        CheckUnsupportedSteamInstallations
        echo "Steam is not installed, run ${bold}sudo apt install steam${normal} to install the native package."
        exit 1
      fi
      continue
    fi
    if [[ ${installed_packages[@]} != *$package* ]]; then
      echo "$package is not installed, run ${bold}sudo $pm_install $package${normal} to install."
      exit 1
    fi
  done
}

# Checking if Steam is installed
function CheckSteamInstall {
  if [[ ${installed_packages[@]} == *"steam"* ]]; then
    echo "Native installation of Steam found."
    set_paths_for steam
  else
    echo "Steam is not installed, run ${bold}sudo apt install steam${normal} to install the native package."
    exit 1
  fi
}

function CheckAssettoProcess {
  ac_pid="$(pgrep "AssettoCorsa.ex")"
  if [[ $ac_pid != "" ]]; then
    Ask "Assetto Corsa is running. Stop Assetto Corsa to proceed?" && kill "$ac_pid" &&
    return
    exit
  fi
}

function CheckTempDir {
  if [[ -d "temp/" ]]; then
    echo "\"temp\" directory found inside current directory. It needs to be removed or renamed for this script to work."
    Ask "Move \"temp/\" to trash?" && gio trash "temp" --force && return
    exit 1
  fi
}

function FindAC {
  # Find installation
  if [ -f "$STEAM_LIBRARY_VDF" ]; then  
    # Extract Steam library paths
    PATH_LIST=$(grep 'path' "$STEAM_LIBRARY_VDF" | awk -F'"' '{print $4}')

    while IFS= read -r LIBRARY_PATH; do
      [[ -z "$LIBRARY_PATH" ]] && continue
      if [[ -d "$LIBRARY_PATH/steamapps" ]]; then
        steamapps_path="$LIBRARY_PATH/steamapps"
      else
        steamapps_path="$LIBRARY_PATH"
      fi
      AC_COMMON="${steamapps_path}/common/assettocorsa"

      if [ -d "$AC_COMMON" ]; then
        echo "Found ${bold}$AC_COMMON${normal}."
        Ask "Is that the right installation?" &&
        set_paths_for assettocorsa "$AC_COMMON" &&
        return
      fi
    done <<< "$PATH_LIST"
  else
    echo "No steam library file found at: $STEAM_LIBRARY_VDF"
  fi
  
  echo "Could not find Assetto Corsa in the default path."
  echo "Please install Assetto Corsa via Steam (steamapps/common/assettocorsa) and rerun this script."
  exit 1
}

function StartMenuShortcut {
  link_file="$AC_COMPATDATA/pfx/drive_c/users/steamuser/AppData/Roaming/Microsoft/Windows/Start Menu/Programs/Content Manager.lnk"
  if [[ -f "$link_file" ]]; then
    echo "Start Menu Shortcut for Content Manager found. This might be causing crashes on start-up."
    Ask "Delete the shortcut?" && rm "$link_file" 
  fi
}

function CheckPrefix {
  if [ -d "$AC_COMPATDATA/pfx" ]; then
    echo "Found existing Wineprefix, deleting it may solve AC not launching/crashing."
    Ask "Delete existing Wineprefix and Content Manager? (preserves configs, presets and mods)" && RemovePrefix
  fi
}
function RemovePrefix {
  # Asking whether to get rid of previous configs
  if [[ -d "ac_configs/" ]]; then
    while :; do
      echo "Found previous save of AC and CM configs in ${bold}$PWD/ac_configs/${normal}."
      Ask "Delete previous saves to proceed?" &&
      rm -r "ac_configs/" &&
      break
      exit 1
    done
  fi
  # Saving configs
  mkdir "ac_configs/"
  ac_config_dir="$AC_COMPATDATA/pfx/drive_c/users/steamuser/Documents/Assetto Corsa"
  cm_config_dir="$AC_COMPATDATA/pfx/drive_c/users/steamuser/AppData/Local/AcTools Content Manager"
  if [[ -d "$ac_config_dir" ]]; then
    while :; do
      echo "Saving AC configs and presets..." &&
      cp -r "$ac_config_dir" "ac_configs" &&
      break
      Error "Failed to copy AC configuration to 'temp', aborting deletion of Wineprefix."
    done
  fi
  if [[ -d "$cm_config_dir" ]]; then
    while :; do
      echo "Saving CM configs and presets..." &&
      cp -r "$cm_config_dir" "ac_configs" &&
      break
      Error "Failed to copy CM configuration to 'temp', aborting deletion of Wineprefix"
    done
  fi
  # Deleting Wineprefix
  if [[ -d "$AC_COMPATDATA/pfx" ]]; then
    while :; do
      echo "Deleting Wineprefix..." &&
      rm -rf "$AC_COMPATDATA" &&
      break
      Error "Failed to delete '$AC_COMPATDATA/pfx'"
    done
  fi
  # Copying back the saved configs
  declare -i copied=0
  if [[ -d "ac_configs/Assetto Corsa" ]]; then
    while :; do
      echo "Copying saved AC configs and presets..." &&
      mkdir -p "$AC_COMPATDATA/pfx/drive_c/users/steamuser/Documents" &&
      cp -r "ac_configs/Assetto Corsa" "$ac_config_dir" &&
      copied+=1 &&
      break
      Error "Failed to copy preserved CM configuration."
    done
  fi
  if [[ -d "ac_configs/AcTools Content Manager" ]]; then
    while :; do
      echo "Copying saved CM configs and presets..." &&
      mkdir -p "$AC_COMPATDATA/pfx/drive_c/users/steamuser/AppData/Local" &&
      cp -r "ac_configs/AcTools Content Manager" "$cm_config_dir" &&
      copied+=1 &&
      break
      Error "Failed to copy preserved CM configuration."
    done
  fi
  # Deleting the saved configs
  if [[ -d "ac_configs/" ]] && (( $copied == 2 )); then
    while :; do
      rm -r "ac_configs/" &&
      break
      Error "Could not delete 'ac_configs/' directory"
    done
  fi
  # Deleting Content Manager
  ac_exe="$AC_COMMON/AssettoCorsa.exe"
  ac_original_exe="$AC_COMMON/AssettoCorsa_original.exe"
  if [[ -f "$ac_original_exe" ]]; then
    while :; do
      echo "Removing AC executable..." &&
      rm "$ac_exe" &&
      mv "$ac_original_exe" "$ac_exe" &&
      break
      Error "Failed to delete Content Manager executable"
    done
  fi
}

function CheckProtonGE {
  ProtonGE="ProtonGE $GE_version"
  echo "$ProtonGE is the latest tested version that works. Using any other version may not work."

  local install_locations=()
  if [[ ${#COMPAT_TOOLS_DIRS[@]} -gt 0 ]]; then
    install_locations=("${COMPAT_TOOLS_DIRS[@]}")
  else
    install_locations=("$COMPAT_TOOLS_DIR")
  fi

  local found_installation=1
  for compat_dir in "${install_locations[@]}"; do
    if [[ -d "$compat_dir/GE-Proton$GE_version" ]]; then
      found_installation=0
      break
    fi
  done

  if (( ! found_installation )); then
    Ask "Reinstall $ProtonGE?" && InstallProtonGE
  else
    Ask "Install $ProtonGE?" && InstallProtonGE
  fi
}

function restore_protonge_backups {
  local entry install_dir backup_dir
  for entry in "$@"; do
    IFS='::' read -r install_dir backup_dir <<< "$entry"
    if [[ -d "$backup_dir" ]]; then
      rm -rf "$install_dir"
      mv "$backup_dir" "$install_dir"
    fi
  done
}

function InstallProtonGE {
  local -a install_locations
  if [[ ${#COMPAT_TOOLS_DIRS[@]} -gt 0 ]]; then
    install_locations=("${COMPAT_TOOLS_DIRS[@]}")
  else
    install_locations=("$COMPAT_TOOLS_DIR")
  fi

  local temp_dir="temp"
  local archive_path="$temp_dir/GE-Proton$GE_version.tar.gz"
  local extracted_dir="$temp_dir/GE-Proton$GE_version"
  local ProtonGE="ProtonGE $GE_version"
  local -a backups=()

  # Prepare temporary workspace
  rm -rf "$temp_dir"
  mkdir -p "$temp_dir" || {
    Error "Failed to create temporary directory for ProtonGE installation"
  }

  echo "Downloading $ProtonGE..."
  if ! wget -q -O "$archive_path" "https://github.com/GloriousEggroll/proton-ge-custom/releases/download/GE-Proton$GE_version/GE-Proton$GE_version.tar.gz"; then
    rm -rf "$temp_dir"
    Error "Failed to download $ProtonGE"
  fi

  if ! tar -xzf "$archive_path" -C "$temp_dir/"; then
    rm -rf "$temp_dir"
    Error "Failed to extract $ProtonGE archive"
  fi

  echo "Installing $ProtonGE..."

  # Ensure compatibility tool directories exist and back up existing installations
  local compat_dir install_dir backup_dir
  for compat_dir in "${install_locations[@]}"; do
    mkdir -p "$compat_dir" || {
      rm -rf "$temp_dir"
      Error "Failed to create compatibility tools directory at $compat_dir"
    }

    install_dir="$compat_dir/GE-Proton$GE_version"
    backup_dir="${install_dir}.backup"

    if [[ -d "$install_dir" ]]; then
      echo "Creating backup of existing GE-Proton$GE_version installation in $compat_dir..."
      rm -rf "$backup_dir"
      if ! mv "$install_dir" "$backup_dir"; then
        rm -rf "$temp_dir"
        restore_protonge_backups "${backups[@]}"
        Error "Failed to back up existing GE-Proton$GE_version installation in $compat_dir"
      fi
      backups+=("$install_dir::$backup_dir")
    fi
  done

  for compat_dir in "${install_locations[@]}"; do
    if ! cp -rfa "$extracted_dir" "$compat_dir"; then
      rm -rf "$temp_dir"
      restore_protonge_backups "${backups[@]}"
      Error "Failed to install $ProtonGE into $compat_dir"
    fi
  done

  rm -rf "$temp_dir"

  for entry in "${backups[@]}"; do
    IFS='::' read -r install_dir backup_dir <<< "$entry"
    rm -rf "$backup_dir"
  done

  echo "${bold}To enable ProtonGE for Assetto Corsa:
 1. Restart Steam
 2. Go to Assetto Corsa > Properties > Compatability
 3. Turn on 'Force the use of a specific Steam Play compatability tool'
 4. From the drop-down, select $ProtonGE.${normal}"
}

function CheckContentManager {
  if [[ -f "$AC_COMMON/AssettoCorsa_original.exe" ]]; then
    Ask "Reinstall Content Manager?" && InstallContentManager
  else
    Ask "Install Content Manager?" && InstallContentManager
  fi
}
function InstallContentManager {
  # Installing CM
  while :; do
    echo "Installing Content Manager..." &&
    wget -q "https://acstuff.club/app/latest.zip" -P "temp/" &&
    unzip -q "temp/latest.zip" -d "temp/" &&
    mv "temp/Content Manager.exe" "temp/AssettoCorsa.exe" &&
    mv -n "$AC_COMMON/AssettoCorsa.exe" "$AC_COMMON/AssettoCorsa_original.exe" &&
    rm "temp/latest.zip" &&
    cp -r "temp/"* "$AC_COMMON/" &&
    rm -r "temp" &&
    break
    Error "Content Manager installation failed"
  done
  # Installing fonts
  while :; do
    echo "Installing fonts required for Content Manager..." &&
    wget -q "https://files.acstuff.ru/shared/T0Zj/fonts.zip" -P "temp/" &&
    unzip -qo "temp/fonts.zip" -d "temp/" &&
    rm "temp/fonts.zip" &&
    cp -r "temp/system" "$AC_COMMON/content/fonts/" &&
    rm -r "temp/" &&
    break
    Error "Font installation for CM failed"
  done
  # Creating symlink
  while :; do
    echo "Creating symlink..." &&
    link_from="$LOCAL/share/Steam/config/loginusers.vdf" &&
    link_to="$AC_COMPATDATA/pfx/drive_c/Program Files (x86)/Steam/config/loginusers.vdf" &&
    ln -sf "$link_from" "$link_to" &&
    break
    Error "Failed to create the symlink for CM"
  done
  # Adding ability to open acmanager uri links
  if [[ -f "$AC_DESKTOP" ]]; then
    mimelist="$HOME/.config/mimeapps.list"
    while :; do
      # Cleaning up previous modifications to mimeapps.list
      if [[ -f "$mimelist" ]]; then
        while :; do
          sed "s|x-scheme-handler/acmanager=Assetto Corsa.desktop;||g" -i "$mimelist" &&
          sed "s|x-scheme-handler/acmanager=Assetto Corsa.desktop||g" -i "$mimelist" &&
          sed '$!N; /^\(.*\)\n\1$/!P; D' -i "$mimelist" &&
          break
          Error "Could not clean up previous modifications to mimeapps.list"
        done
      fi
      # Adding acmanager to mimeapps.list
      echo "Adding ability to open acmanager links..." &&
      sed "s|steam steam://rungameid/244210|$APPLAUNCH_AC|g" -i "$AC_DESKTOP" &&
      gio mime x-scheme-handler/acmanager "Assetto Corsa.desktop" 1>& /dev/null &&
      break
      Error "Could not add acmanager to mimeapps.list"
    done
  else
    echo "Assetto Corsa does not have a .desktop shortcut, URI links to CM will not work."
  fi
  echo "When starting Content Manager, set the root Assetto Corsa folder to ${bold}Z:$AC_COMMON${normal}"
}

function CheckCSP {
  # Getting CSP version
  data_manifest_file="$AC_COMMON/extension/config/data_manifest.ini"
  if [[ -f "$data_manifest_file" ]]; then
    current_CSP_version="$(cat "$data_manifest_file" | grep "SHADERS_PATCH=" | sed 's/SHADERS_PATCH=//g')"
  fi
  # Asking whether to install
  if [[ $current_CSP_version == "" ]]; then
    Ask "Install CSP (Custom Shaders Patch) v$CSP_version?" && InstallCSP
  elif [[ $current_CSP_version == "$CSP_version" ]]; then
    Ask "Reinstall CSP v$CSP_version?" && InstallCSP
  else
    Ask "CSP v$current_CSP_version is already installed. Install CSP v$CSP_version instead?" && InstallCSP
  fi
}
function InstallCSP {
  # Adding dwrite dll override
  reg_dwrite="$(echo "$(cat "$AC_COMPATDATA/pfx/user.reg")" | grep "dwrite")"
  if [[ $reg_dwrite == "" ]]; then
    while :; do
      echo "Adding DLL override 'dwrite'..." &&
      sed '/\"\*d3d11"="native\"/a \"dwrite"="native,builtin\"' "$AC_COMPATDATA/pfx/user.reg" -i &&
      break
      Error "Could not create DLL override for 'dwrite'"
    done
  else
    echo "DLL override 'dwrite' already exists."
  fi
  # Installing CSP
  while :; do
    echo "Downloading CSP..." &&
    rm -rf "temp" &&
    mkdir -p "temp" &&
    wget -q -O "temp/lights-patch-v$CSP_version.zip" "https://acstuff.club/patch/?get=$CSP_version" &&
    echo "Installing CSP..." &&
    unzip -qo "temp/lights-patch-v$CSP_version.zip" -d "temp/" &&
    rm "temp/lights-patch-v$CSP_version.zip" &&
    cp -r "temp/." "$AC_COMMON" &&
    rm -r "temp" &&
    break
    Error "CSP installation failed"
  done
  # Installing fonts for CSP
  while :; do
    echo "Installing fonts required for CSP... (this might take a while)"
    protontricks 244210 corefonts 1>& /dev/null &&
    return
    Error "Could not install corefonts for CSP"
  done
}

function HardLinkFile {
  local src="$1"
  local dest="$2"

  if [[ ! -f "$src" ]]; then
    echo "Source file '$src' not found, skipping."
    return 1
  fi

  local dest_dir
  dest_dir="$(dirname "$dest")"
  mkdir -p "$dest_dir" || return 1

  ln -f "$src" "$dest" || return 1
}

function HardLinkTree {
  local src="$1"
  local dest="$2"

  if [[ ! -d "$src" ]]; then
    echo "Source directory '$src' not found, skipping."
    return 1
  fi

  mkdir -p "$dest" || return 1

  local dotglob_state nullglob_state
  dotglob_state="$(shopt -p dotglob)"
  nullglob_state="$(shopt -p nullglob)"

  shopt -s dotglob nullglob

  local item base
  for item in "$src"/*; do
    [[ -e "$item" ]] || continue
    base="$(basename "$item")"
    if [[ -d "$item" ]]; then
      HardLinkTree "$item" "$dest/$base" || {
        eval "$dotglob_state"
        eval "$nullglob_state"
        return 1
      }
    elif [[ -f "$item" ]]; then
      HardLinkFile "$item" "$dest/$base" || {
        eval "$dotglob_state"
        eval "$nullglob_state"
        return 1
      }
    fi
  done

  eval "$dotglob_state"
  eval "$nullglob_state"

  return 0
}

function LinkBridgeConfig {
  local config_src="${PWD}/assetto_corsa_configs/assetto_corsa_bridge_controls.ini"
  local config_dest="${AC_COMPATDATA}/pfx/drive_c/users/steamuser/Documents/Assetto Corsa/cfg/assetto_corsa_bridge_controls.ini"
  local action
  local action_ing

  if [[ ! -f "$config_src" ]]; then
    echo "Bridge control preset not found at $config_src, skipping."
    return
  fi

  if [[ -f "$config_dest" ]]; then
    action="Reinstall"
    action_ing="Reinstalling"
  else
    action="Install"
    action_ing="Installing"
  fi

  Ask "$action Assetto Corsa ROS Control Preset?" || return

  while :; do
    echo "$action_ing bridge control preset to '$config_dest'..." &&
    HardLinkFile "$config_src" "$config_dest" &&
    break
    Error "Failed to hard link the bridge control preset"
  done
}

function LinkPlugin {
  local plugin_src="${PWD}/assetto_corsa_plugin"
  local plugin_dest="${AC_COMMON}/apps/python/assetto_corsa_plugin"
  local action
  local action_ing

  if [[ ! -d "$plugin_src" ]]; then
    echo "Plugin directory not found at $plugin_src, skipping."
    return
  fi

  if [[ -d "$plugin_dest" ]]; then
    action="Reinstall"
    action_ing="Reinstalling"
  else
    action="Install"
    action_ing="Installing"
  fi

  Ask "$action Assetto Corsa ROS Plugin?" || return

  while :; do
    echo "$action_ing plugin to '$plugin_dest'..." &&
    HardLinkTree "$plugin_src" "$plugin_dest" &&
    break
    Error "Failed to hard link the Assetto Corsa ROS plugin"
  done
}

function LinkVehicles {
  local vehicles_src="${PWD}/assetto_corsa_vehicles"
  local vehicle
  local vehicle_name
  local vehicle_dest
  local action="Install"
  local action_ing

  if [[ ! -d "$vehicles_src" ]]; then
    echo "Vehicle directory not found at $vehicles_src, skipping."
    return
  fi

  for vehicle in "$vehicles_src"/*; do
    [[ -d "$vehicle" ]] || continue
    vehicle_name="$(basename "$vehicle")"
    vehicle_dest="${AC_COMMON}/content/cars/${vehicle_name}"
    if [[ -d "$vehicle_dest" ]]; then
      action="Reinstall"
      break
    fi
  done

  if [[ $action == "Reinstall" ]]; then
    action_ing="Reinstalling"
  else
    action_ing="Installing"
  fi

  Ask "$action Assetto Corsa ROS Vehicle Content?" || return

  for vehicle in "$vehicles_src"/*; do
    [[ -d "$vehicle" ]] || continue
    vehicle_name="$(basename "$vehicle")"
    vehicle_dest="${AC_COMMON}/content/cars/${vehicle_name}"
    while :; do
      echo "$action_ing vehicle '${vehicle_name}' to '$vehicle_dest'..." &&
      HardLinkTree "$vehicle" "$vehicle_dest" &&
      break
      Error "Failed to hard link vehicle '${vehicle_name}'"
    done
  done
}

# Helper functions
function Error {
  echo "${bold}ERROR${normal}: $1. If this is an issue, please report it on github."
  exit 1
}
function Ask {
  echo
  local yn
  while true; do
    if ! read -r -p "$* [y/n]: " yn < /dev/tty; then
      echo "${bold}ERROR${normal}: Unable to read user input." >&2
      exit 1
    fi
    case $yn in
      [Yy]*) return 0 ;;
      [Nn]*) return 1 ;;
    esac
  done
}

function check_generated_files {
  if [ ! -d "$AC_COMPATDATA/pfx/drive_c/Program Files (x86)/Steam/config" ]; then
    echo "Before proceeding, please do the following to generate a Wineprefix:
 1. Launch Assetto Corsa with Proton-GE $GE_version
 2. Wait until Assetto Corsa launches (it takes a while)
 3. Exit Assetto Corsa
Then start the script again and skip the steps relating to deleting the Wineprefix and installing ProtonGE."
    exit 1
  fi
}

# Checking stuff
CheckOS
CheckUnsupportedSteamInstallations
CheckDependencies
CheckSteamInstall
CheckAssettoProcess
CheckTempDir
# Running functions
FindAC
StartMenuShortcut
CheckPrefix
CheckProtonGE
# Checking if assettocorsa's files were generated
check_generated_files
# Continuing to run functions
CheckContentManager
CheckCSP
LinkPlugin
LinkVehicles
LinkBridgeConfig
echo "${bold}All done!${normal}"