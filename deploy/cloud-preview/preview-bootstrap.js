'use strict';
(() => {
  const prefix = '/_cloud-preview/';
  const nativeFetch = window.fetch.bind(window);
  function scopedUrl(input) {
    const url = new URL(input, location.href);
    if (url.origin === location.origin && !url.pathname.startsWith(prefix)) {
      url.pathname = prefix.slice(0, -1) + url.pathname;
    }
    return url.href;
  }
  window.fetch = (input, options) => {
    if (input instanceof Request) {
      return nativeFetch(new Request(scopedUrl(input.url), input), options);
    }
    return nativeFetch(scopedUrl(input), options);
  };
})();
