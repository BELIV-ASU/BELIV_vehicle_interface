![image](https://github.com/BELIV-ASU/Dataspeed_ULCnode/assets/123104450/7e3014ab-a083-433e-be22-6e1f3d1b9a2b)

# Beliv_Vehicle_Interface

Vehicle Interface for Dataspeed FORD Mach_E

## Citation
Guo, Hengcong, et al. Developing an Automated Vehicle Research Platform by Integrating Autoware with the DataSpeed Drive-By-Wire System. No. 2024-01-1981. SAE Technical Paper, 2024.
```
@techreport{guo2024developing,
  title={Developing an Automated Vehicle Research Platform by Integrating Autoware with the DataSpeed Drive-By-Wire System},
  author={Guo, Hengcong and Li, Jiangtao and Saravanan, Nithish Kumar and Wishart, Jeffrey and Zhao, Junfeng},
  year={2024},
  institution={SAE Technical Paper}
}
```
## Getting Started

* Plase read through the Autoware documentation first before you do any modifcation to the folder. [Click here](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/overview/#3-create-a-vehicle_interface-package)

* If you know git version control works, please feel free to working on in right away, if you are not familiar with the version control,Plase watch this video and pratice few before you upload your file to this Repo [Click here](https://www.youtube.com/watch?v=USjZcfj8yxE)

## Overall Structure
![image](https://github.com/BELIV-ASU/Dataspeed_ULCnode/assets/123104450/73878cb2-976a-4372-9162-225c3325fa80)

## To Sync your local Repo

Generate a Personal Access Token (PAT) on GitHub:
* Go to your GitHub account settings.
* Select "Developer settings" > "Personal access tokens."
* Click "Generate new token" and give it appropriate permissions (repo, read/write access, etc.).
* Copy the generated token.
Clone the Repository Using the Classic PAT:
* In your terminal, use the following command to clone the repository, replacing your-token with your actual classic personal access token:
```
git clone https://username:your-token@github.com/BELIV-ASU/Dataspeed_ULCnode.git
```
*** Replace "username" with your Github username

*** Reoakce "your_token" with your personal access token you generated
Navigate to the rood directory:
```
cd repository/specific-folder
```
then follow the command below:
```
git add .
git commit -m "Sync specific folder"
git push origin master
```
*** Replace "master" with the specfic brunch you want to modify

## Authors

ASU Believ Lab

## Version History

* 0.2
    * Files in final version need to be uploaded
    * See [commit change]() or See [release history]()
* 0.1
    * Initial Release

## License

This project is licensed under the [ASU Beliv Lab] License - see the LICENSE.md file for details
