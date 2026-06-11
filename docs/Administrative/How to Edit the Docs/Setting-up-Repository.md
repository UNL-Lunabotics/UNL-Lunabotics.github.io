---
title: Setting up the Repository
parent: How to Edit the Docs
nav_order: 1
---

## Setting up the Repository

To make changes on any lunabotics repository, you will need to set it up locally by cloning it. Cloning, if unfamiliar, just means to copy the repository to your local machine. You will then make changes on that copy, and push them up to the [UNL-Lunabotics GitHub](https://github.com/unl-lunabotics) organization.

Before we clone the documentation, ensure you have followed the [Setup Guide for VSCode]({% link docs/Non-Programmer Friendly Zone/How-to-Use-VSCode.md %}).

### Cloning

To clone this repository using git, type the following command into a terminal:

```bash
git clone https://github.com/unl-lunabotics/unl-lunabotics.github.io
```

A folder should now be created in the directory the terminal is pointed to, named "unl-lunabotics.github.io".

### Opening it in VSCode

Now that the repository is cloned, you can use VSCode to edit it. Open VSCode and press the "Open Folder" button in the Get Started window. Or you can navigate to `File > Open Folder` in the top navigation bar. Find the cloned folder and open it.

### Downloading Suggested Extensions

After opening the project in VSCode for the first time, you may be prompted to download some recommended extensions for the workspace. It is **highly** recommended that you download these as it includes the linter (which helps keep our formatting consistent) and some additional helpful Markdown tools.

Opening the project in VSCode, you will be prompted to download the recommended extensions for the workspace. I highly recommend saying yes as it includes the linter that helps keep our formatting consistent, and it also includes some helpful markdown tools.

### Making a new Branch

To start editing, you will want to create a new branch, as we generally do not allow editing on the main branch. Using git on the terminal, type in the following command, replacing `your-new-branch-name` with a short name that describes the feature, fix, or change you plan to make:

```bash
git switch -c your-new-branch-name
```

This will create a new branch on your **local** copy of the repository. To push the branch to GitHub, type in the following git command:

```bash
git push -u origin your-new-branch-name
```

### Committing changes

Once a branch is created, you are able to edit or add to this repository as you see fit, and commit them. To commit changes, type the following git command:

```bash
git add .
git commit -m "Message here"
```

Replace the commit message (shown in quotes above) with a short descriptive message about your changes. Good convention is to commit regularly so that commit messages represent the changes.

### Using VSCode Source Control

Instead of using git commands, you may prefer a graphical interface to use. VSCode has a Source Control view, which is the third icon in your sidebar. Click on the icon, and a new view should pop up. You can track your changes, type in a commit message, and commit to the repository.

You can also do more advanced interactions by hovering the mouse over the "Changes" tab and clicking the three dots in the right-hand side, and selecting an option.

{: .important}
Note that this view is more limiting than raw git commands, but using this view can simplify basic interactions with git, such as tracking and committing changes.

Once the repository is set up, learn how to [Edit the Repository]({% link docs/Administrative/How to Edit the Docs/Editing the Repository/index.md %}).

> Author: Aiden Kimmerling <https://github.com/TheKing349>.
