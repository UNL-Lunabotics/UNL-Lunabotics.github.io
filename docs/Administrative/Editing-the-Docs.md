---
title: Editing the Docs
parent: Administrative
nav_order: 1
---

<!-- #TODO: update for new folder layout and broken links -->

## Table of contents

{:toc}

## How to Edit the Docs

This serves as a guide for editing these docs. Follow the Table of Contents to get started.

{: .important}
It is **highly** recommended using VSCode to edit. Firstly, it is just easier to navigate and edit a lot of files in VSCode rather than the browser. You can work on multiple pages at once before having to do one commit and the GitHub repository includes folder organization for convenience. Additionally, VSCode will also include the recommended extensions for this repository, **including the linter that we use and some convenience features for editing in Markdown**.

## Setting up the Repository

To make changes on any lunabotics repository, you will need to set it up locally by cloning it. Cloning, if unfamiliar, just means to copy the repository to your local machine. You will then make changes on that copy, and push them up to the [UNL-Lunabotics GitHub](https://github.com/unl-lunabotics) organization.

Before we clone the documentation, ensure you have followed the [Setup Guide for VSCode]({% link docs/Administrative/How-to-Use-VSCode.md %}).

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

Once the repository is set up, learn how to [Edit the Repository]({% link docs/Administrative/Docs-Reference.md %}).

## Testing Locally

While you're editing, it may be helpful to get a preview of the website. To locally test, you will need to use the command line on most systems.

### Installing Dev Tools

First, download and install all [prerequisites](https://jekyllrb.com/docs/installation/). Then, use the following command in a terminal to install [jekyll](https://jekyllrb.com/docs/), a program suite used for GitHub Pages:

```bash
gem install jekyll bundler
```

This will install jekyll on your system.

### Run the Server

Now open a new terminal in VSCode. Type the following command to run:

```bash
bundle exec jekyll serve --livereload
```

Then point your browser to <http://127.0.0.1:4000>. You should see the website. If any changes are made to the code, refresh the browser to show updates.

{: .note}
If you do not want the server to reload as you edit files, omit the `--livereload` flag

### Troubleshooting

On macOS (and maybe other platforms), it seems like Ruby does not configure your environment path fully. If you get the error `command not found: bundle` or `command not found: jekyll`, it is likely you need to update your environment path. On macOS and Linux, open a new terminal, and type in the following command:

```bash
gem env | grep "EXECUTABLE DIRECTORY"
```

This lists the correct path that `jekyll` and `bundler` is in.

Copy the path that shows. Example: `/opt/homebrew/lib/ruby/gems/4.0.0/bin`

Now, update your environment PATH, using the following:

```bash
nano ~/.zshrc
```

{: .note}
On Linux, replace `.zshrc` with `.bashrc`.

`nano` is a terminal text editor. Use arrow keys to move the cursor all the way to the bottom. Add the following:

```bash
# Ruby and Gem Installs
export PATH=$PATH:PATH_GOES_HERE
```

Replace `PATH_GOES_HERE` with the copied output from earlier. Example:
`export PATH=$PATH:/opt/hombrew/lib/ruby/gems/4.0.0/bin`

Now, press `CTRL+X` and `y` to save. Go back to VSCode and close the current terminal. Open a new one and try running `bundle exec jekyll serve` again.

## Rules

To keep consistency and readability throughout documentation, a few rules are enforced. Most of the rules are explained in [The Linter]({% link docs/Administrative/The-Linter.md %}) documentation.

### Local Testing

In addition, we also require local testing. This is important as testing your changes locally serves as a visual way of proofreading, which can help with readability, clarity, and consistency.

If needed, refer back to the [Testing Locally](#local-testing) documentation.

### Pull Requests and Branches

Finally, this repository enforces a pull-request workflow instead of committing to main directly, by creating a new branch. All this means is that, in order for your changes to get pushed to the main website, it must be approved first. This ensures high quality documentation and is more akin to industry workflows.

It is also important to note that general convention is that you commit changes **regularly** on a branch. Do not have a giant commit. Rather, commit once a single "unit" or code is written. This helps keep commit history readable and helps if a revision or revert is needed.

### AI Policy

It is the firm belief of the programming team leadership that generative AI, while sometimes helpful, can be easily overused and end up creating slop that's more damaging than helpful. Additionally, the process of writing documentation and explaining it to people helps build knowledge, understanding, communication skills, and looks better on a resume. Because of these reasons, writing documentation with generative AI is prohibited. It is acceptable to use generative AI as a research aid, or to assist in creating example code or graphs (it's pretty good at mermaid). No text should be written by AI. At minimum, information gathered with the use of generative AI should be paraphrased after thoughtful review of what it's actually telling you, as it is frequently wrong or misleading.

## How to Push Changes

Using git and GitHub, there are several ways to push your changes to a repository. You can commit changes directly to the main (or master) branch, which is usually not preferred.

Instead, you will interact with this repository (and most other lunabotics repos) by creating a new branch that describes a feature or fix. You will then commit your changes on this branch. Once the feature or fix is complete, you will create what's called a Pull Request.

### What is a Pull Request?

A Pull Request is just a way for your branch to be merged to the main branch, pushing all your changes at once. This is preferred as it also allows for external review by other programmers to ensure quality, correctness, and tests, before changes are pushed to the main branch.

For more information about what a Pull Request is, look at the official GitHub [documentation](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests).

### How to Create a Pull Request

To create a Pull Request, go to this repository on GitHub and follow the official GitHub [documentation](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request).

{: .note}
There is a VSCode extension named GitHub Pull Requests author GitHub that allows you to create/review PR's in the IDE. This is the recommended way to review PR's as it lets more easily see code changes and use the markdown preview functionality and website preview abilities.

Once the Pull Request is submitted, it will be reviewed by other programmers. They will accept it or give suggestions or improvements to make the code and result better.

After your Pull Request is accepted, your changes will be merged to the main branch. If you plan to keep making edits, feel free to keep this branch up and active, and submit a new Pull Request if needed. Otherwise, if the feature or fix is complete, you may delete the branch on GitHub.

### Conclusion

Congratulations! You have successfully edited the documentation and pushed your changes using Pull Requests! If you have any questions, comments, or concerns, feel free to hesitate to any programming (or non-programming!) member for assistance. We are always here to help you learn and grow wherever we can! Have fun documenting!

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
