import torchviz 

def visualize_computation_graph(target_tensor, save_dir, view=False):
    grad = torchviz.make_dot(target_tensor)
    grad.render(save_dir, format='pdf', view=view)