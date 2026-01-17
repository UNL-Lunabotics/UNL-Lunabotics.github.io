---
title: How to Edit the Docs
parent: Administrative
nav_order: 2
---

# How to Edit the Docs

This will be a *basic* overview on how Just-the-Docs works. For a more in-depth look, view their [documentation](https://just-the-docs.com)

## Project Structure

There is a lot going on in this repo, so let's break it down.

### The `docs` Folder

This is the main part of this repo, and where you will likely be spending the most time in. Within the docs folder, there's a specific structure we follow:

Each "Section" will have a folder associated with it. Within each folder, there are markdown files. The `index.md` is the main file and will create an entry in the Sidebar. Other files inside the folder will be treated as "children" of the Sidebar entry, and rendered **underneath** it.

## YAML Tags

When you create a new file, you must use a special format for the Sidebar to know where the documentation file goes:

```yaml
---
title: My Awesome Documentation
parent: Administrative
nav_order: 4
---

# ACTUAL Documentation here
```

In this example, the name or `title` of the page is "My Awesome Documentation". This name shows on the Sidebar.

Then, it has a `parent` of "Administrative". This means it will render **underneath** the `Administrative` Sidebar entry.

Lastly, it has a `nav_order` of 4. This means it will be the 4th entry under "Administrative"

A documentation page does not **need** a `parent` or `nav_order`. At the very least, provide a `title`, so it gets rendered as a separate entry in Sidebar.

{: .important }
The folder name or structure does not matter. It exists for repository organization. The names will all tie back to `title`. So in our previous example, if we changed the `title` entry of `index.md`, the Sidebar name would've changed.<br> Likewise, just because a markdown file exists in a folder does **not** mean it will be a child in the Sidebar. You **must** use the `parent` tag for that.

## Adding Links to Another Doc Page

If we needed to reference another documentation page, say the [Administrative]({% link docs/Administrative/index.md %}) page, we don't link the URL.

Instead, we use the following syntax:

{% raw %}

```markdown
[Administrative]({% link docs/Administrative/index.md %})
```

{% endraw %}

## Testing Locally

To test changes locally you'll need to install jekyll. Follow the [docs](https://jekyllrb.com/docs/) on how to do so.

Then enter the following in the VSCode terminal(if one is not opened, enter the keybind ``Ctrl+Shift+` ``):

```console
bundle install
bundle exec jekyll serve
```

Then point your browser to <http://127.0.0.1:4000>.

If you make a change, wait for your console to refresh, then manually refresh the page.

Have fun documenting!

> Author: Aiden Kimmerling (<https://github.com/TheKing349>)
