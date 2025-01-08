# Step 1: Change to the catkin workspace directory
cd ./catkin_ws

# Step 2: Run catkin build to build the workspace
catkin build

cd ../

# Step 3: Create a temporary directory in /tmp if it doesn't already exist
mkdir -p ./tmp/extracted

# Step 4: Copy the zip file to /tmp directory (update with the actual path to the zip file)
cp ./Simulation.zip ./tmp/

# Step 5: Navigate to /tmp and extract the zip file into the newly created temp folder
cd tmp/
unzip Simulation.zip -d extracted

cd ../

# Step 6: Move extracted contents to the target directory in your catkin workspace
mv -f ./tmp/extracted/* ./catkin_ws/devel/lib/simulation/

chmod +x ./catkin_ws/devel/lib/simulation/Simulation.x86_64

# Step 7: Optionally, clean up temporary files
rm -r ./tmp
