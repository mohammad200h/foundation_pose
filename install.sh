cd foundation_pose/weights
gdown --folder https://drive.google.com/drive/folders/1BEQLZH69UO5EOfah-K9bfI3JyP9Hf7wC
gdown --folder https://drive.google.com/drive/folders/12Te_3TELLes5cim1d7F7EBTwUSe7iRBj
cd ../..

 pip install torch==2.5.1 torchaudio==2.5.1 torchvision==0.20.1
 pip install aiohttp==3.10.10 albumentations==1.4.18 cv_bridge
 pip install Cython==3.0.10 cycler dearpygui==1.11.1
 pip install einops==0.8.0 glfw==2.7.0 gdown
 pip install h5py==3.13.0 hickle==5.0.3 imageio==2.35.1
 pip install joblib==1.4.2 kornia==0.5.10 kiwisolver
 pip install loguru==0.7.2 matplotlib==3.9.2 Ninja
 pip install "numpy<2" omegaconf==2.3.0 open3d==0.18.0
 pip install openai==1.51.2 opencv_contrib_python==4.10.0.84 opencv_python==4.10.0.82
 pip install opencv_python_headless==4.10.0.84 pandas==2.2.3 Pillow==11.0.0
 pip install progressbar33==2.4 psutil PyAutoGUI==0.9.54
 pip install PyOpenGL==3.1.0 pyparsing pyrender==0.1.45
 pip install PySide6==6.8.0.1 pytest==8.2.2 pytinyrenderer==0.0.14
 pip install pytorch_lightning==2.4.0 PyYAML==6.0.2 Requests==2.32.3
 pip install ruamel.base==1.0.0 ruptures==1.1.9 scikit_learn==1.5.2
 pip install scipy==1.14.1 setuptools==72.1.0 six
 pip install tqdm==4.66.5 transformations trimesh==4.4.7
 pip install ultralytics==8.3.31 warp==1.0.4 warp_lang==1.3.1
 pip install websocket_client==1.8.0 yacs==0.1.8
 pip install setuptools==70.0.0
 pip install "git+https://github.com/facebookresearch/pytorch3d.git@stable"
 pip install pybind11==2.10.1
 pip ruamel_yaml==0.18.10
 pip install ninja
 sudo apt install -y ninja-build

cd foundation_pose/nvdiffrast
pip install .