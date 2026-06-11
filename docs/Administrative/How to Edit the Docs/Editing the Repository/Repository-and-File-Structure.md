---
title: Repository and File Structure
parent: Editing the Repository
nav_order: 1
---

## Repository and File Structure

For this guide, it will be helpful to have the repository open as you read. There's a lot going on in this repository, so let's break it down.

### The docs Folder

This is the main part of this repo, and where you will likely be spending the most time in. Within the docs folder, there's a specific structure just-the-docs uses:

Each Section of this website has a folder associated with it in the repository, such as "Administrative" or "Curriculum". Within each of those folders, there are Markdown files. The `index.md` file is the main documentation and will create an entry in the Sidebar. Other files inside the folder will be treated as "children" of the Sidebar entry, and rendered **underneath** it.

### The attachments Folder

The attachments folder is used to store any image or other files (zips, etc.) that is linked to a piece of documentation.

### Other configuration files

There are other files used for just-the-docs configuration. These files should generally be untouched unless asked to edit, as it can break the website if set up incorrectly.

The `_config.yml` file is the main just-the-docs config, specifying the website's name and description, as well as links and sidenotes. For more information, refer to the just-the-docs [documentation](https://just-the-docs.com/docs/configuration/).

### File Structure

While our documentation uses plain Markdown, there is a specific structure to follow when making a new file.

#### YAML Header

Just-the-docs requires a special header to configure the Sidebar and render pages. The YAML header is as follows:

```yaml
---
title: My Awesome Documentation
parent: Administrative
nav_order: 4
---
```

In this example, the name of the page is "My Awesome Documentation", defined by the `title` tag. This is the name that shows up on the Sidebar.

Then, it is a part of the "Administrative" page, as marked by the `parent`.

Lastly, it has a `nav_order` of 4. This means it will be the 4th entry under the parent "Administrative" page.

All pages will need a `title`. A `parent` or `nav_order` is optional, but there for organizational purposes.

Any page with children will render a Table of Contents on the parent page.

It is important to note that the folder name and structure is separate from the YAML header. Folder structure is there just for repository organization. The website will **only** use the YAML header to decide where to render pages and what the names will be.

This also means that you will **need** to add the YAML header to a Markdown file for the website to render it. Similarly, a page will render at the root of the Sidebar unless a `parent` tag is set.

For additional information, refer to the just-for-docs [documentation]([LINK](https://just-the-docs.com/docs/navigation/)).

#### Headings and Subheadings

After the YAML header, you will want to introduce the documentation topic. Here our convention is to use a subheading to start, then describe what the file's purpose is:

```markdown
## My Awesome Heading

This guide serves to provide an example of how our documentation works.
```

Feel free to change this wording to your liking.

Then you can write the rest of the file using plain Markdown.

#### Author Tag

At the end of all documents (excluding `index.md` files), we require the use of an Author Tag. This is a signature of sorts. We want you to be able to go into industry and point to anything you wrote in any lunabotics repository and say that you made it.

An example Author Tag is below:

```markdown
> Author: Firstname Lastname <https://github.com/your-username>.
```

All together, a new file should look something like the following:

```markdown
---
title: My Awesome Documentation
parent: Administrative
nav_order: 4
---

## My Awesome Documentation

This guide serves to provide an example of how our documentation works.

### First Subsection

This is a subsection.

#### First Sub-subsection

This is a sub-subsection.

### Second Subsection

This is *another* subsection.

> Author: Firstname Lastname <https://github.com/my-username>.

```

Once you feel comfortable with how the project and files are laid out, move on to specific [just-the-docs features]({% link docs/Administrative/How to Edit the Docs/Editing the Repository/Just-the-Docs-Features.md %}).

> Author: Aiden Kimmerling <https://github.com/TheKing349>
