from arguments import ModelParams
from scene import Scene
from gaussian_renderer import GaussianModel
from arguments import ModelParams, get_combined_args, ArgumentParser, Namespace


# Set up command line argument parser
parser = ArgumentParser(description="Testing script parameters")
model = ModelParams(parser, sentinel=True)
dataset = model.extract(
    Namespace(
        sh_degree=3,
        source_path="/home/jennyw2/data/test_data/robot_iphone",
        model_path="/home/jennyw2/data/output/robot_iphone",
        images="images",
        resolution=-1,
        white_background=False,
        data_device="cuda",
        eval=False
    )
)

# 2. Create a GaussianModel (degree matches your config)
gaussians = GaussianModel(dataset.sh_degree)

# 3. Create the Scene
scene = Scene(dataset, gaussians, load_iteration=-1, shuffle=False, num_cams=3)

# 4. Get the training cameras
train_cameras = scene.getTrainCameras()
print(train_cameras)