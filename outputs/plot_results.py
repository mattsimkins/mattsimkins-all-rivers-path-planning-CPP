import matplotlib.pyplot as plt
import matplotlib.image as mpimg

 
def read_traj(file_name):
    
    '''Reads in trajectories for training. Trajectories are structured as
    two, space separated floating point values for x and y respectivel,
    followed by an end line. This function parces characters and formats
    them for use by PVF.
    
    Args:
    
        file_path: Gives the relative path to the directory containing
        trajectory files for training, String.
        
        file_name: Gives the trajectory name for the .txt file being
        read in as a trajectory, String.
    
    Returns:

        traj: A data type structured for use by PVF, List[Tuple(Float, Float),
        Tuple(Float, Float), Tuple(Float, Float), ...].
    '''
    
    try:
        traj_file = open(file_name, "r")
        traj = []
        content = traj_file.readlines()
        for line_char in content:
            x_done = False
            if line_char == "":
                break
            x_value = ""
            y_value = ""
            for one_char in line_char:
                if one_char == " ":
                    x_done = True
                    pass
                if x_done == False:
                    x_value = x_value + one_char
                if x_done:
                    y_value = y_value + one_char
            traj.append((float(x_value), float(y_value)))
            traj_file.close()
    
    except FileNotFoundError:
        print("Trajectory file not found!")
        return None

    except ValueError:
            print(('Trajectory not correctly formatted and will be ignored.'
                   'Should be of the form:\n<x1> <y1>\n<x2> <y2>\n...'))
            return None
        
    return traj    
    

def plot_trajectory(*args, title="data", **kwargs):
    '''Plots a trajectory. The title is optional. The trajectory could be from
    task space or grid space. This is purely a visualization tool.
    
    Kwargs:
    
        title: Optional plot title, None, or String.
        
        extents: Specify plot extents. If None, the plot is
        automatically scaled to fit the trajectory. This is given as x-min,
        x-max, y-min, y-max, List[Float, Float, Float, Float]
        
        image_file: Specifies the image file name to overlay trajectories 
        onto. The image file must be located in the working directory, and it
        was tested using .jpg image types. This feature is usefull for
        visualizing trajectory obsticals, such as, roads or walls. However, 
        the image extents must be properly sized and alligned to the task
        space where the trajectory was generaged, String. 
    
    Args:

        Trajectory to plot. Could originate from task space or grid
        space, List[tuple(Float, Float), tuple(Float, Float), ...].
                
    Returns:
        
        N/A
    '''
    
    #Allows user to specify coordinate ext
    extents = kwargs.get("extents")
    
    #Allows user to specify a background image
    img_name = kwargs.get("image_file")   
    
    if img_name != None:
        img = mpimg.imread(img_name)   
        
        # Create a figure and axes
        fig, ax = plt.subplots()
        ax.imshow(img, extent=extents)
        
    else:
        ax = plt

    #Add plots of trajectories
    colors = ["blue", "orange", "green", "red", "purple", "brown", "pink", "gray", "olive", "cyan"] 
    marker_shape = ["o", "s", "^", "v", "o", "s", "*", "D", "h", "p"]
    mark_ind = 0
    for data in args:
        for i in range(len(data) -1): 
            x1 = data[i][0]
            y1 = data[i][1]
            x2 = data[i+1][0]
            y2 = data[i+1][1]
            ax.plot([x1, x2], [y1, y2], c=colors[mark_ind])
            ax.scatter(x1, y1, c=colors[mark_ind], marker=marker_shape[mark_ind])
        mark_ind += 1
        if mark_ind + 1 > len(colors):
            mark_ind = 0
        
    #Set extents of plots
    if extents != None:
        plt.xlim(extents[0], extents[1])
        plt.ylim(extents[2], extents[3])
    
    plt.title(title)
    plt.show()
    plt.close()


def main():
    traj0 = read_traj("estimated_path_from_start.txt")
    traj1 = read_traj("estimated_path_from_dead_end.txt")
    traj2 = read_traj("estimated_path_from_intersection.txt")
    plot_trajectory(traj0, traj1, traj2, 
                    extents=[-2200, 2200, -1300, 1300],
                    image_file = "Maze.jpg")

if __name__ == "__main__":
    main()