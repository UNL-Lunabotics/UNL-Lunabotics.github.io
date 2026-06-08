---
title: How to Push Changes
parent: How to Edit the Docs
nav_order: 6
---

## How to Push Changes

Using git and GitHub, there are several ways to push your changes to a repository. You can commit changes directly to the main (or master) branch, which is usually not preferred.

Instead, you will interact with this repository (and most other lunabotics repos) by creating a new branch that describes a feature or fix. You will then commit your changes on this branch. Once the feature or fix is complete, you will what's called a Pull Request.

### What is a Pull Request?

A Pull Request is just a way for your branch to be merged to the main branch, pushing all your changes at once. This is preferred as it also allows for external review by other programmers to ensure quality, correctness, and tests, before changes are pushed to the main branch.

For more information about a Pull Request, look at the official GitHub [documentation](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests).

### How to Make a new Branch

To start, you will want to create a new branch. Using git on the terminal, type in the following command, replacing `your-new-branch-name` with a short name that describes the feature, fix, or change you plan to make:

```bash
git switch -c your-new-branch-name
```

This will create a new branch on your **local** copy of the repository. To push the branch to GitHub, type in the following git command:

```bash
git push -u origin your-new-branch-name
```

### Committing changes

Once a branch is created, you are able to edit or add to this repository as you see fit. If needed, refer back to [How to Edit the Docs]({% link docs/Administrative/How to Edit the Docs/Editing the Repository/index.md %}).

As the [Rules]({% link docs/Administrative/How to Edit the Docs/Rules.md %}) state, make sure you commit regularly.

### How to Create a Pull Request

To create a Pull Request, go to this repository on GitHub and follow the official GitHub [documentation](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request).

Once the Pull Request is submitted, it will be reviewed by other programmers. They will accept it or give suggestions or improvements to make the code and result better.

After your Pull Request is accepted, your changes will be merged to the main branch. If you plan to keep making edits, feel free to keep this branch up and active, and submit a new Pull Request if needed. Otherwise, if the feature or fix is complete, you may delete the branch on GitHub.

### Using VSCode Source Control

Instead of using git commands, you may prefer a graphical interface to use. VSCode has a Source Control view, which is the third icon in your sidebar. Click on the icon, and a new view should pop up. You can track your changes, type in a commit message, and commit to the repository.

You can also do more advanced interactions by hovering the mouse over the "Changes" tab and clicking the three dots in the right-hand side, and selecting an option.

{: .important}
Note that this view is more limiting than raw git commands, but using this view can simplify basic interactions with git, such as tracking and committing changes.

### Conclusion

Congratulations! You have successfully edited the documentation and pushed your changes using Pull Requests! If you have any questions, comments, or concerns, feel free to hesitate to any programming (or non-programming!) member for assistance. We are always here to help you learn and grow wherever we can!

> Author: Aiden Kimmerling <https://github.com/TheKing349>
