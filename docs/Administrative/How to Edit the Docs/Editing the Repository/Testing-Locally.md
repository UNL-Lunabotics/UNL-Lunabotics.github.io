---
title: Testing Locally
parent: Editing the Repository
nav_order: 3
---

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

Now that you know the basics about creating and editing documentation, please read the additional conventions and [Rules]({% link docs/Administrative/How to Edit the Docs/Rules.md %}) we enforce.

> Author: Aiden Kimmerling <https://github.com/TheKing349>
