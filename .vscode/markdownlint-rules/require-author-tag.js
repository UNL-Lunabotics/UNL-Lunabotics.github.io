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

    const validIndex = info.footerIndices.find(i =>
      authorRx.test(lines[i].trim())
    );

    if (validIndex !== undefined) return;

    if (info.authorTagIndex !== undefined) {
      const i = info.authorTagIndex;

      onError({
        lineNumber: i + 1,
        detail: "Malformed author footer.",
        context: lines[i],
        fixInfo: replaceLineFix(i, template)
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
