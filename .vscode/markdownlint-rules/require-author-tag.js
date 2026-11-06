const {
  getFooterInfo,
  createFooterTagFix,
  replaceLineFix
} = require("./footer-utils");

/** @type {import("markdownlint").Rule} */
module.exports = {
  names: ["require-author-tag"],
  description: "File must end with one or more '> Author: … <github link>' lines.",
  information: new URL(
    "https://unl-lunabotics.github.io/docs/Administrative/The-Linter.html#custom-rule-required-author-tag"
  ),
  tags: ["credit"],
  parser: "none",
  function: (params, onError) => {
    const path = require("path");
    if (path.basename(params.name) === "index.md") return;

    const { lines } = params;
    const info = getFooterInfo(params);

    // > Author: First Last (<https://github.com/handle>)
    const authorRx =
      /^>\s*Author:\s+[A-Z][\w.'-]*(?:\s+[A-Z][\w.'-]*)*\s+\(<https:\/\/github\.com\/[\w.-]+>\)$/;

    const template = "> Author: First Last (<https://github.com/your-handle>)";

    const authorIndices = info.footerIndices.filter(i =>
      /^>\s*Author:/.test(lines[i].trim())
    );

    const validIndices = authorIndices.filter(i =>
      authorRx.test(lines[i].trim())
    );

    if (authorIndices.length > 0 && validIndices.length === authorIndices.length) return;

    if (authorIndices.length > 0) {
      const invalidIndex = authorIndices.find(i =>
        !authorRx.test(lines[i].trim())
      );

      onError({
        lineNumber: invalidIndex + 1,
        detail: "Malformed author footer.",
        context: lines[invalidIndex],
        fixInfo: replaceLineFix(invalidIndex, template)
      });

      return;
    }

    // The combined rule handles a document with neither tag.
    if (!info.hasAnyTag) return;

    onError({
      lineNumber: Math.max(info.end + 1, 1),
      detail: "Missing author footer.",
      context: lines[Math.max(info.end, 0)] || "",
      fixInfo: createFooterTagFix(params, "author", template)
    });
  }
};
