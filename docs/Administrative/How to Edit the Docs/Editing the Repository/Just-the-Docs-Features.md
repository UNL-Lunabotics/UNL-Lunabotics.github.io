---
title: Just-the-Docs Features
parent: Editing the Repository
nav_order: 2
---

## Just-the-Docs Features

Using just-the-docs, there are a few convenience features commonly used.

### Callouts

Callouts are side notes that will render differently. For example, the following is a callout:

{: .note}
This is an example callout.

To use a callout in your documentation, use the following syntax:

```markdown
{: .note}
This is an example callout.
```

Callouts are defined in the `_config.yml` file. Currently, the supported callouts are:

- highlight
- important
- new
- note
- warning

To use another callout, simply replace the `.note` for another callout. Example:

```markdown
{: .important}
Super important text
```

renders as such:

{: .important}
Super important text.

Each of these callouts use a different color, also defined in the `_config.yml` file. For more information, refer to the just-the-docs [documentation](https://just-the-docs.com/docs/ui-components/callouts/).

### Linking Media

It is common to need to hyperlink a website, attachment, or another piece of documentation.

#### Hyperlinking a Website

For a basic hyperlink, follow the default Markdown syntax:

```markdown
[Text](https://google.com)
```

This renders the following: [Text](https://google.com). The text in brackets is what is rendered on the website.

#### Hyperlinking Documentation

Sometimes you may want to link another piece of our documentation With just-the-docs, there is a bit of a different way to link a piece of documentation. Use the following syntax:

{% raw %}

```markdown
[Administrative]({% link docs/Administrative/index.md %})
```

{% endraw %}

This renders the following: [Administrative]({% link docs/Administrative/index.md %})

#### Attaching an Image

To attach an image, it is similar syntax to hyperlinking a piece of documentation:

{% raw %}

```markdown
![Alt Text]({% link attachments/example.png %})
```

{% endraw %}

This renders the following:

![Alt Text]({% link attachments/example.png %})

Be sure to set the Alt Text in brackets as it helps accessibility for screen readers.

Once you understand commonly used features, learn how to [locally test this repository]({% link docs/Administrative/How to Edit the Docs/Editing the Repository/Testing-Locally.md %}).

> Author: Aiden Kimmerling <https://github.com/TheKing349>
