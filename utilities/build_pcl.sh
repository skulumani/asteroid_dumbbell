# Build the Point Cloud Library

echo "This will download and build PCL"
read -p "Press ENTER to continue"

sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl -y
sudo apt-get update -y
sudo apt-get install libpcl-dev -y

git clone https://github.com/strawlab/python-pcl.git ~/python-pcl
cd ~/python-pcl
python setup.py build_ext -i
python setup.py install
