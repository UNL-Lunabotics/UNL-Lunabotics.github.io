
const {
  getFooterInfo,
  createFooterTagFix
} = require("./footer-utils");

/** @type {import("markdownlint").Rule} */
module.exports = {
  names: ["require-last-reviewed-tag"],
  description: "File must end with a last-reviewed date footer tag.",
  information: new URL(
    "https://unl-lunabotics.github.io/docs/Administrative/The-Linter.html#custom-rule-required-last-reviewed-tag"
  ),
  tags: ["credit"],
  parser: "none",
  function: (params, onError) => {
    const path = require("path");
    if (path.basename(params.name) === "index.md") return;

    const { lines } = params;
    const info = getFooterInfo(params);

      // > Last Reviewed: YYYY-MM-DD
    const lastReviewedRx =
      /^>\s*Last Reviewed:\s+\d{4}-(0[1-9]|1[0-2])-(0[1-9]|[12]\d|3[01])$/;

    const template = "> Last Reviewed: YYYY-MM-DD";

    const validIndex = info.footerIndices.find(i =>
      lastReviewedRx.test(lines[i].trim())
    );

    if (validIndex !== undefined) return;

    if (info.lastReviewedTagIndex !== undefined) {
      const i = info.lastReviewedTagIndex;

      onError({
        lineNumber: i + 1,
        detail: "Malformed last-reviewed date; use YYYY-MM-DD.",
        context: lines[i]
      });

      return;
    }

    // The combined rule handles a document with neither tag.
    if (!info.hasAnyTag) return;

    onError({
      lineNumber: Math.max(info.end + 1, 1),
      detail: "Missing last-reviewed footer.",
      context: lines[Math.max(info.end, 0)] || "",
      fixInfo: createFooterTagFix(
        params,
        "last-reviewed",
        template
      )
    });
  }
};
