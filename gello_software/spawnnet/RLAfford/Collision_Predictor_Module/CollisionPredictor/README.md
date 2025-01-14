# Where2Act: From Pixels to Actions for Articulated 3D Objects

![Overview](/images/teaser.png)

**The Proposed Where2Act Task.** Given as input an articulated 3D object, we learn to propose the actionable information for different robotic manipulation primitives (e.g. pushing, pulling): (a) the predicted actionability scores over pixels; (b) the proposed interaction trajectories, along with (c) their success likelihoods, for a selected pixel highlighted in red. We show two high-rated proposals (left) and two with lower scores (right) due to interaction orientations and potential robot-object collisions.

## Introduction
One of the fundamental goals of visual perception is to allow agents to meaningfully interact with their environment. In this paper, we take a step towards that long-term goal -- we extract highly localized actionable information related to elementary actions such as pushing or pulling for articulated objects with movable parts. For example, given a drawer, our network predicts that applying a pulling force on the handle opens the drawer. We propose, discuss, and evaluate novel network architectures that given image and depth data, predict the set of actions possible at each pixel, and the regions over articulated parts that are likely to move under the force. We propose a learning-from-interaction framework with an online data sampling strategy that allows us to train the network in simulation (SAPIEN) and generalizes across categories.

## About the paper

Where2Act is accepted to ICCV 2021!

Our team: 
[Kaichun Mo](https://cs.stanford.edu/~kaichun),
[Leonidas J. Guibas](https://geometry.stanford.edu/member/guibas/),
[Mustafa Mukadam](http://www.mustafamukadam.com/),
[Abhinav Gupta](http://www.cs.cmu.edu/~abhinavg/),
and [Shubham Tulsiani](https://shubhtuls.github.io/)
from 
Stanford University and FaceBook AI Research.

Arxiv Version: https://arxiv.org/abs/2101.02692

Project Page: https://cs.stanford.edu/~kaichun/where2act

## Citations
    
    @inProceedings{mo2021where2act,
        title={Where2Act: From Pixels to Actions for Articulated 3D Objects},
        author={Mo, Kaichun and Guibas, Leonidas and Mukadam, Mustafa and Gupta, Abhinav and Tulsiani, Shubham},
        year={2021},
        booktitle={International Conference on Computer Vision (ICCV)}
    }

## About this repository

This repository provides data and code as follows.


```
    data/                   # contains data, models, results, logs
    code/                   # contains code and scripts
         # please follow `code/README.md` to run the code
    stats/                  # contains helper statistics
```

## Questions

Please post issues for questions and more helps on this Github repo page. We encourage using Github issues instead of sending us emails since your questions may benefit others.

## License

MIT Licence

## Updates

* [Jan 15, 2021] Preliminary version of Data and Code released. For more code on evaluation, stay tuned.
* [Aug 31, 2021] Fixed a bug: https://github.com/daerduoCarey/where2act/blob/main/code/models/model_3d.py#L147-L151. Thanks to Ruihai Wu for pointing this out!

